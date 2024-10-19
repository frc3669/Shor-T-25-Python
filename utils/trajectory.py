import json
import cmath
from datetime import timedelta

class Sample:
    def __init__(self, timestamp, position, heading, velocity, angular_velocity):
        self.timestamp = timestamp
        self.position = position
        self.heading = heading
        self.velocity = velocity
        self.angular_velocity = angular_velocity

class Trajectory:
    def __init__(self, filename: str):
        with open(filename, 'r') as file:
            jsonData = json.load(file)
            
            self.samples = []
            for item in jsonData['samples']:
                timestamp = item['timestamp']
                position = complex(item['x'], item['y'])
                heading = item['heading']
                velocity = complex(item['velocityX'], item['velocityY'])
                angular_velocity = item['angularVelocity']
                
                sample = Sample(timestamp, position, heading, velocity, angular_velocity)
                self.samples.append(sample)
            
            self.end_time = self.samples[-1].timestamp if self.samples else 0

    def get_sample(self, index):
        if index < len(self.samples):
            return self.samples[index]
        else:
            return Sample(0, 0, 0, 0, 0)

    def get_sample_count(self):
        return len(self.samples)

    def get_end_time(self):
        return self.end_time