import json, wpilib
from datetime import timedelta

class Sample:
    def __init__(self, timestamp, position, heading, velocity, angular_velocity, acceleration, angular_acceleration):
        self.timestamp = timestamp
        self.position = position
        self.heading = heading
        self.velocity = velocity
        self.angular_velocity = angular_velocity
        self.acceleration = acceleration
        self.angular_acceleration = angular_acceleration

class Trajectory:
    def __init__(self, filename: str):
        """creates a trajectroy object from the givien filename from the deploy directory"""
        with open(wpilib.getDeployDirectory() + "/" + filename, 'r') as file:
            jsonData = json.load(file)

            self.samples = []
            for item in jsonData['trajectory']['samples']:
                timestamp = item['t']
                position = complex(item['x'], item['y'])
                heading = item['heading']
                velocity = complex(item['vx'], item['vy'])
                angular_velocity = item['omega']
                acceleration = complex(item['ax'], item['ay'])
                angular_acceleration = item['alpha']
                
                sample = Sample(timestamp, position, heading, velocity, angular_velocity, acceleration, angular_acceleration)
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