#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import math
import time

class EnvironmentalSensorTestPublisher(Node):
    def __init__(self):
        super().__init__('environmental_sensor_test_publisher')
        
        # Create publishers for all environmental sensors
        self.temperature_pub = self.create_publisher(Float32, '/environmental/temperature', 10)
        self.humidity_pub = self.create_publisher(Float32, '/environmental/humidity', 10)
        self.methane_pub = self.create_publisher(Float32, '/environmental/methane', 10)
        self.co_pub = self.create_publisher(Float32, '/environmental/carbon_monoxide', 10)
        self.no2_pub = self.create_publisher(Float32, '/environmental/nitrogen_dioxide', 10)
        self.pm25_pub = self.create_publisher(Float32, '/environmental/pm25', 10)
        self.pm10_pub = self.create_publisher(Float32, '/environmental/pm10', 10)
        self.aqi_pub = self.create_publisher(Float32, '/environmental/air_quality_index', 10)
        
        # Timer to publish data every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_sensor_data)
        
        # Initialize base values for realistic simulation
        self.base_temp = 25.0  # Base temperature in Celsius
        self.base_humidity = 50.0  # Base humidity percentage
        self.time_offset = 0.0
        
        self.get_logger().info('Environmental Sensor Test Publisher started')
        self.get_logger().info('Publishing on topics:')
        self.get_logger().info('  - /environmental/temperature')
        self.get_logger().info('  - /environmental/humidity')
        self.get_logger().info('  - /environmental/methane')
        self.get_logger().info('  - /environmental/carbon_monoxide')
        self.get_logger().info('  - /environmental/nitrogen_dioxide')
        self.get_logger().info('  - /environmental/pm25')
        self.get_logger().info('  - /environmental/pm10')
        self.get_logger().info('  - /environmental/air_quality_index')

    def publish_sensor_data(self):
        current_time = time.time()
        self.time_offset += 0.1
        
        # Generate realistic temperature data (20-35°C with daily variation)
        daily_temp_variation = 5.0 * math.sin(self.time_offset * 0.1)  # Slow daily cycle
        temp_noise = random.uniform(-2.0, 2.0)
        temperature = self.base_temp + daily_temp_variation + temp_noise
        temperature = max(15.0, min(40.0, temperature))  # Clamp between 15-40°C
        
        # Generate realistic humidity data (30-80% with inverse correlation to temperature)
        humidity_base = 70.0 - (temperature - 20.0) * 1.5  # Inverse correlation with temp
        humidity_noise = random.uniform(-10.0, 10.0)
        humidity = humidity_base + humidity_noise
        humidity = max(20.0, min(90.0, humidity))  # Clamp between 20-90%
        
        # Generate gas sensor data
        # Methane (MQ-4): Usually low, occasional spikes
        methane_base = 5.0 + random.uniform(0, 15.0)
        if random.random() < 0.05:  # 5% chance of spike
            methane_base += random.uniform(20.0, 40.0)
        methane = max(0.0, methane_base)
        
        # Carbon Monoxide (MICS-4514): Usually very low
        co_base = 0.5 + random.uniform(0, 2.0)
        if random.random() < 0.02:  # 2% chance of spike
            co_base += random.uniform(5.0, 15.0)
        co = max(0.0, co_base)
        
        # Nitrogen Dioxide (MICS-4514): Usually low
        no2_base = 0.01 + random.uniform(0, 0.05)
        if random.random() < 0.03:  # 3% chance of spike
            no2_base += random.uniform(0.1, 0.3)
        no2 = max(0.0, no2_base)
        
        # Particulate Matter PM2.5 (DSM501B)
        pm25_base = 8.0 + random.uniform(0, 12.0)
        weather_factor = 1.0 + 0.3 * math.sin(self.time_offset * 0.05)  # Weather variation
        pm25 = pm25_base * weather_factor
        pm25 = max(0.0, min(150.0, pm25))  # Clamp between 0-150 μg/m³
        
        # Particulate Matter PM10 (DSM501B) - Always higher than PM2.5
        pm10 = pm25 + random.uniform(5.0, 20.0)
        pm10 = max(pm25, min(200.0, pm10))  # Ensure PM10 > PM2.5
        
        # Calculate Air Quality Index based on PM2.5 (simplified EPA formula)
        if pm25 <= 12.0:
            aqi = (pm25 / 12.0) * 50.0
        elif pm25 <= 35.4:
            aqi = 50.0 + ((pm25 - 12.0) / (35.4 - 12.0)) * 50.0
        elif pm25 <= 55.4:
            aqi = 100.0 + ((pm25 - 35.4) / (55.4 - 35.4)) * 50.0
        elif pm25 <= 150.4:
            aqi = 150.0 + ((pm25 - 55.4) / (150.4 - 55.4)) * 50.0
        else:
            aqi = 200.0 + min(((pm25 - 150.4) / 100.0) * 100.0, 100.0)
        
        # Create and publish messages
        temp_msg = Float32()
        temp_msg.data = float(temperature)
        self.temperature_pub.publish(temp_msg)
        
        humidity_msg = Float32()
        humidity_msg.data = float(humidity)
        self.humidity_pub.publish(humidity_msg)
        
        methane_msg = Float32()
        methane_msg.data = float(methane)
        self.methane_pub.publish(methane_msg)
        
        co_msg = Float32()
        co_msg.data = float(co)
        self.co_pub.publish(co_msg)
        
        no2_msg = Float32()
        no2_msg.data = float(no2)
        self.no2_pub.publish(no2_msg)
        
        pm25_msg = Float32()
        pm25_msg.data = float(pm25)
        self.pm25_pub.publish(pm25_msg)
        
        pm10_msg = Float32()
        pm10_msg.data = float(pm10)
        self.pm10_pub.publish(pm10_msg)
        
        aqi_msg = Float32()
        aqi_msg.data = float(aqi)
        self.aqi_pub.publish(aqi_msg)
        
        # Log data periodically
        if int(current_time) % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(f'Environmental Data:')
            self.get_logger().info(f'  Temperature: {temperature:.1f}°C')
            self.get_logger().info(f'  Humidity: {humidity:.1f}%')
            self.get_logger().info(f'  Methane: {methane:.1f} ppm')
            self.get_logger().info(f'  CO: {co:.2f} ppm')
            self.get_logger().info(f'  NO2: {no2:.3f} ppm')
            self.get_logger().info(f'  PM2.5: {pm25:.1f} μg/m³')
            self.get_logger().info(f'  PM10: {pm10:.1f} μg/m³')
            self.get_logger().info(f'  AQI: {aqi:.0f}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        publisher = EnvironmentalSensorTestPublisher()
        
        print("🌍 Environmental Sensor Test Publisher")
        print("=====================================")
        print("Publishing realistic environmental sensor data...")
        print("Sensors simulated:")
        print("  📊 SHT30 Temperature & Humidity Sensor")
        print("  🔥 MQ-4 Methane Gas Detection")
        print("  ☠️  MICS-4514 CO/NO2 Sensor")
        print("  🌪️  DSM501B PM2.5/PM10 Dust Sensor")
        print("  🌍 Air Quality Index Calculator")
        print("\nPress Ctrl+C to stop...")
        
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        print("\n🛑 Shutting down Environmental Sensor Test Publisher...")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'publisher' in locals():
            publisher.destroy_node()
        rclpy.shutdown()
        print("✅ Environmental Sensor Test Publisher stopped")

if __name__ == '__main__':
    main()
