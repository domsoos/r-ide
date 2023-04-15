import sys
import rosbag

def main(argv):
    if len(argv) != 2:
        print("Usage: python3 rosbag_duration.py <path_to_rosbag>")
        sys.exit(1)

    bag_path = argv[1]

    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            duration = bag.get_end_time() - bag.get_start_time()
            print(duration)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main(sys.argv)