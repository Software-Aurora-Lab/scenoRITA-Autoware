import zipfile
import argparse
from pathlib import Path

import gdown

"""
Author: Changnam Hong
"""


def download_and_extract_zip(id: str, output_dir_name: str = "rosbags") -> Path:
    zip_file = gdown.download(id=id)
    zip_file_path = Path(zip_file)

    sample_dir_path = zip_file_path.parent / Path(output_dir_name)
    sample_dir_path.mkdir(parents=True, exist_ok=True)

    output_dir = sample_dir_path / Path(zip_file_path.stem)

    with zipfile.ZipFile(zip_file, 'r') as zip_ref:
        zip_ref.extractall(output_dir)

    zip_file_path.unlink()

    return output_dir


if __name__ == "__main__":
    # Usage: python3 download_rosbags.py <id1> <id2> <id3> ...

    parser = argparse.ArgumentParser(description="Download and extract a ZIP file from a given URL.")
    parser.add_argument("identifiers", nargs="+", help="List of File identifiers")

    args = parser.parse_args()

    for id in args.identifiers:
        output = download_and_extract_zip(id)
        print(f"Downloaded zip file is extracted and saved in {output}")
