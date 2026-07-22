import csv
import os
import re


class CSVLogger:
    def __init__(self, save_directory: str, file_prefix: str, fieldnames: list):
        self.save_directory = save_directory
        self.file_prefix = file_prefix
        self.fieldnames = fieldnames

        self.file = None
        self.writer = None
        self.trajectory_id = None

        os.makedirs(self.save_directory, exist_ok=True)

    def get_next_trajectory_id(self):

        pattern = rf"{self.file_prefix}_(\d+)\.csv"

        ids = []

        for filename in os.listdir(self.save_directory):
            match = re.match(pattern, filename)

            if match:
                ids.append(int(match.group(1)))

        if not ids:
            return 1

        return max(ids) + 1

    def start(self, filename: str | None = None):
        if filename is None:
            self.trajectory_id = self.get_next_trajectory_id()
            filename = f"{self.file_prefix}_{self.trajectory_id:06d}.csv"
        else:
            self.trajectory_id = None

        filepath = os.path.join(self.save_directory, filename)

        self.file = open(filepath, "w", newline="")

        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)

        self.writer.writeheader()

        return self.trajectory_id, filepath

    def write(self, row):
        if self.writer is not None:
            self.writer.writerow(row)

    def flush(self):
        if self.file is not None:
            self.file.flush()

    def stop(self):
        if self.file is not None:
            self.file.flush()
            self.file.close()

        self.file = None
        self.writer = None
        self.trajectory_id = None
