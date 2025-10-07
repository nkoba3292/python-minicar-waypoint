from path_planning import compute_path
from visualize import animate_path

if __name__ == "__main__":
    path = compute_path()
    if path:
        animate_path(path)
