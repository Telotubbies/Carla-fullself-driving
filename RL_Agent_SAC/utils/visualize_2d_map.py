import sys
import os
import argparse
import carla
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.map_2d_visualizer import create_2d_map_from_carla
import logging
logging.basicConfig(level=logging.INFO)
def main():
    parser = argparse.ArgumentParser(description='Create 2D topview map from CARLA')
    parser.add_argument('--host', default='localhost', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--output', default='carla_2d_map.png', help='Output image path')
    parser.add_argument('--town', default=None, help='Town to load (e.g., Town01_Opt)')
    args = parser.parse_args()
    print("=" * 60)
    print("🗺️  CARLA 2D Topview Map Visualizer")
    print("=" * 60)
    print(f"Connecting to CARLA at {args.host}:{args.port}...")
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        if args.town:
            print(f"Loading town: {args.town}")
            world = client.load_world(args.town)
        else:
            world = client.get_world()
        print(f"✅ Connected to CARLA")
        print(f"   World: {world.get_map().name}")
        print("")
        print("Creating 2D map...")
        visualizer = create_2d_map_from_carla(world, args.output)
        print("")
        print("=" * 60)
        print(f"✅ 2D Map created successfully!")
        print(f"   Output: {args.output}")
        print(f"   Size: {visualizer.width}x{visualizer.height} pixels")
        print("=" * 60)
    except RuntimeError as e:
        print(f"❌ Error: {e}")
        print("   Make sure CARLA server is running:")
        print("   cd /home/a/Desktop/CARLA_0.9.16 && ./CarlaUE4.sh -quality-level=Low")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
if __name__ == '__main__':
    main()