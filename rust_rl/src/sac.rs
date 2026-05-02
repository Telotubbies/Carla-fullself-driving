// SAC (Soft Actor-Critic) Algorithm in Rust
// แปลงจาก Python เป็น Rust เพื่อ performance สูงสุด

use tch::{nn, nn::OptimizerConfig, Device, Tensor};
use std::sync::Arc;
use parking_lot::RwLock;

/// Actor Network (Policy)
pub struct Actor {
    net: nn::Sequential,
    mean_linear: nn::Linear,
    log_std_linear: nn::Linear,
    device: Device,
}

impl Actor {
    pub fn new(vs: &nn::Path, obs_dim: i64, action_dim: i64, hidden_dim: i64) -> Self {
        let net = nn::seq()
            .add(nn::linear(vs / "fc1", obs_dim, hidden_dim, Default::default()))
            .add_fn(|x| x.relu())
            .add(nn::linear(vs / "fc2", hidden_dim, hidden_dim, Default::default()))
            .add_fn(|x| x.relu());
        
        let mean_linear = nn::linear(vs / "mean", hidden_dim, action_dim, Default::default());
        let log_std_linear = nn::linear(vs / "log_std", hidden_dim, action_dim, Default::default());
        
        Actor {
            net,
            mean_linear,
            log_std_linear,
            device: vs.device(),
        }
    }
    
    /// Forward pass - คำนวณ mean และ log_std
    pub fn forward(&self, obs: &Tensor) -> (Tensor, Tensor) {
        let features = self.net.forward(obs);
        let mean = self.mean_linear.forward(&features);
        let log_std = self.log_std_linear.forward(&features).clamp(-20.0, 2.0);
        (mean, log_std)
    }
    
    /// Sample action จาก policy
    pub fn sample(&self, obs: &Tensor) -> (Tensor, Tensor, Tensor) {
        let (mean, log_std) = self.forward(obs);
        let std = log_std.exp();
        
        // Sample from normal distribution
        let normal = Tensor::randn_like(&mean);
        let x_t = &mean + &std * &normal;
        
        // Apply tanh squashing
        let action = x_t.tanh();
        
        // Calculate log probability
        let log_prob = self.calculate_log_prob(&mean, &log_std, &x_t, &action);
        
        (action, log_prob, mean.tanh())
    }
    
    fn calculate_log_prob(&self, mean: &Tensor, log_std: &Tensor, x_t: &Tensor, action: &Tensor) -> Tensor {
        // Log probability of normal distribution
        let std = log_std.exp();
        let var = &std * &std;
        
        let log_prob = -((x_t - mean).pow_tensor_scalar(2.0) / (2.0 * &var))
            - log_std
            - ((2.0 * std::f64::consts::PI).ln() / 2.0);
        
        // Correction for tanh squashing
        let log_prob = log_prob.sum_dim_intlist(&[-1i64][..], false, tch::Kind::Float);
        let correction = (1.0 - action.pow_tensor_scalar(2.0) + 1e-6).log().sum_dim_intlist(&[-1i64][..], false, tch::Kind::Float);
        
        log_prob - correction
    }
}

/// Critic Network (Q-function)
pub struct Critic {
    q1: nn::Sequential,
    q2: nn::Sequential,
}

impl Critic {
    pub fn new(vs: &nn::Path, obs_dim: i64, action_dim: i64, hidden_dim: i64) -> Self {
        let q1 = nn::seq()
            .add(nn::linear(vs / "q1_fc1", obs_dim + action_dim, hidden_dim, Default::default()))
            .add_fn(|x| x.relu())
            .add(nn::linear(vs / "q1_fc2", hidden_dim, hidden_dim, Default::default()))
            .add_fn(|x| x.relu())
            .add(nn::linear(vs / "q1_fc3", hidden_dim, 1, Default::default()));
        
        let q2 = nn::seq()
            .add(nn::linear(vs / "q2_fc1", obs_dim + action_dim, hidden_dim, Default::default()))
            .add_fn(|x| x.relu())
            .add(nn::linear(vs / "q2_fc2", hidden_dim, hidden_dim, Default::default()))
            .add_fn(|x| x.relu())
            .add(nn::linear(vs / "q2_fc3", hidden_dim, 1, Default::default()));
        
        Critic { q1, q2 }
    }
    
    pub fn forward(&self, obs: &Tensor, action: &Tensor) -> (Tensor, Tensor) {
        let input = Tensor::cat(&[obs, action], -1);
        let q1 = self.q1.forward(&input);
        let q2 = self.q2.forward(&input);
        (q1, q2)
    }
}

/// SAC Agent
pub struct SACAgent {
    actor: Actor,
    critic: Critic,
    critic_target: Critic,
    
    actor_optimizer: nn::Optimizer,
    critic_optimizer: nn::Optimizer,
    
    alpha: f64,
    alpha_tensor: Tensor,
    log_alpha: Tensor,
    alpha_optimizer: nn::Optimizer,
    
    gamma: f64,
    tau: f64,
    target_entropy: f64,
    
    device: Device,
}

impl SACAgent {
    pub fn new(
        obs_dim: i64,
        action_dim: i64,
        hidden_dim: i64,
        lr: f64,
        gamma: f64,
        tau: f64,
        alpha: f64,
        device: Device,
    ) -> Self {
        // Create variable stores
        let vs_actor = nn::VarStore::new(device);
        let vs_critic = nn::VarStore::new(device);
        let vs_critic_target = nn::VarStore::new(device);
        
        // Create networks
        let actor = Actor::new(&vs_actor.root(), obs_dim, action_dim, hidden_dim);
        let critic = Critic::new(&vs_critic.root(), obs_dim, action_dim, hidden_dim);
        let critic_target = Critic::new(&vs_critic_target.root(), obs_dim, action_dim, hidden_dim);
        
        // Copy critic to target
        vs_critic_target.copy(&vs_critic).unwrap();
        
        // Create optimizers
        let actor_optimizer = nn::Adam::default().build(&vs_actor, lr).unwrap();
        let critic_optimizer = nn::Adam::default().build(&vs_critic, lr).unwrap();
        
        // Alpha (temperature parameter)
        let log_alpha = Tensor::zeros(&[1], (tch::Kind::Float, device)).set_requires_grad(true);
        let alpha_tensor = log_alpha.exp();
        let vs_alpha = nn::VarStore::new(device);
        vs_alpha.root().var_copy("log_alpha", &log_alpha);
        let alpha_optimizer = nn::Adam::default().build(&vs_alpha, lr).unwrap();
        
        // Target entropy
        let target_entropy = -(action_dim as f64);
        
        SACAgent {
            actor,
            critic,
            critic_target,
            actor_optimizer,
            critic_optimizer,
            alpha,
            alpha_tensor,
            log_alpha,
            alpha_optimizer,
            gamma,
            tau,
            target_entropy,
            device,
        }
    }
    
    /// Select action (inference mode)
    pub fn select_action(&self, obs: &Tensor, deterministic: bool) -> Tensor {
        tch::no_grad(|| {
            if deterministic {
                let (mean, _) = self.actor.forward(obs);
                mean.tanh()
            } else {
                let (action, _, _) = self.actor.sample(obs);
                action
            }
        })
    }
    
    /// Update networks
    pub fn update(
        &mut self,
        obs: &Tensor,
        action: &Tensor,
        reward: &Tensor,
        next_obs: &Tensor,
        done: &Tensor,
    ) -> (f64, f64, f64, f64) {
        // Update critic
        let (critic_loss, q1_value, q2_value) = self.update_critic(obs, action, reward, next_obs, done);
        
        // Update actor
        let (actor_loss, entropy) = self.update_actor(obs);
        
        // Update alpha
        let alpha_loss = self.update_alpha(entropy);
        
        // Soft update target network
        self.soft_update_target();
        
        (critic_loss, actor_loss, alpha_loss, (q1_value + q2_value) / 2.0)
    }
    
    fn update_critic(
        &mut self,
        obs: &Tensor,
        action: &Tensor,
        reward: &Tensor,
        next_obs: &Tensor,
        done: &Tensor,
    ) -> (f64, f64, f64) {
        // Calculate target Q value
        let target_q = tch::no_grad(|| {
            let (next_action, next_log_prob, _) = self.actor.sample(next_obs);
            let (target_q1, target_q2) = self.critic_target.forward(next_obs, &next_action);
            let target_q = target_q1.min_other(&target_q2);
            
            // V = Q - alpha * log_pi
            let target_v = &target_q - &self.alpha_tensor * &next_log_prob.unsqueeze(-1);
            
            // Q_target = r + gamma * (1 - done) * V
            reward + self.gamma * (1.0 - done) * target_v
        });
        
        // Current Q values
        let (current_q1, current_q2) = self.critic.forward(obs, action);
        
        // Critic loss (MSE)
        let critic_loss = (&current_q1 - &target_q).pow_tensor_scalar(2.0).mean(tch::Kind::Float)
            + (&current_q2 - &target_q).pow_tensor_scalar(2.0).mean(tch::Kind::Float);
        
        // Backprop
        self.critic_optimizer.zero_grad();
        critic_loss.backward();
        self.critic_optimizer.step();
        
        let q1_mean = f64::from(current_q1.mean(tch::Kind::Float));
        let q2_mean = f64::from(current_q2.mean(tch::Kind::Float));
        
        (f64::from(critic_loss), q1_mean, q2_mean)
    }
    
    fn update_actor(&mut self, obs: &Tensor) -> (f64, f64) {
        let (action, log_prob, _) = self.actor.sample(obs);
        let (q1, q2) = self.critic.forward(obs, &action);
        let q = q1.min_other(&q2);
        
        // Actor loss = alpha * log_pi - Q
        let actor_loss = (&self.alpha_tensor * &log_prob.unsqueeze(-1) - &q).mean(tch::Kind::Float);
        
        // Backprop
        self.actor_optimizer.zero_grad();
        actor_loss.backward();
        self.actor_optimizer.step();
        
        let entropy = f64::from(-log_prob.mean(tch::Kind::Float));
        
        (f64::from(actor_loss), entropy)
    }
    
    fn update_alpha(&mut self, entropy: f64) -> f64 {
        // Alpha loss = -log_alpha * (log_pi + target_entropy)
        let alpha_loss = -&self.log_alpha * (entropy + self.target_entropy);
        
        // Backprop
        self.alpha_optimizer.zero_grad();
        alpha_loss.backward();
        self.alpha_optimizer.step();
        
        // Update alpha tensor
        self.alpha_tensor = self.log_alpha.exp();
        self.alpha = f64::from(&self.alpha_tensor);
        
        f64::from(alpha_loss)
    }
    
    fn soft_update_target(&mut self) {
        // Polyak averaging: θ_target = τ * θ + (1 - τ) * θ_target
        tch::no_grad(|| {
            for (target_param, param) in self.critic_target.q1.parameters().iter()
                .zip(self.critic.q1.parameters().iter()) {
                target_param.copy_(&(self.tau * param + (1.0 - self.tau) * target_param));
            }
            
            for (target_param, param) in self.critic_target.q2.parameters().iter()
                .zip(self.critic.q2.parameters().iter()) {
                target_param.copy_(&(self.tau * param + (1.0 - self.tau) * target_param));
            }
        });
    }
    
    /// Save model
    pub fn save(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        // TODO: Implement model saving
        Ok(())
    }
    
    /// Load model
    pub fn load(&mut self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        // TODO: Implement model loading
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_sac_creation() {
        let agent = SACAgent::new(
            10, // obs_dim
            3,  // action_dim
            256, // hidden_dim
            3e-4, // lr
            0.99, // gamma
            0.005, // tau
            0.2, // alpha
            Device::Cpu,
        );
        
        let obs = Tensor::randn(&[32, 10], (tch::Kind::Float, Device::Cpu));
        let action = agent.select_action(&obs, false);
        
        assert_eq!(action.size(), vec![32, 3]);
    }
}
