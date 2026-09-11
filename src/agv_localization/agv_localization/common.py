from pathlib import Path
import yaml
import numpy as np
from ament_index_python.packages import get_package_share_directory
from .core import gnss_covariance, finite


def load(node):
    share = Path(get_package_share_directory('agv_localization'))
    desc = Path(get_package_share_directory('agv_description'))/'config'
    config = yaml.safe_load(Path(node.declare_parameter('config',str(share/'config/measurements.yaml')).value).read_text())
    platform = yaml.safe_load(Path(node.declare_parameter('platform',str(desc/'platform.yaml')).value).read_text())
    camera = yaml.safe_load(Path(node.declare_parameter('camera_config',str(desc/'linescan.yaml')).value).read_text())
    mode = node.declare_parameter('profile','normal').value
    if mode not in ('normal','zero'):
        raise ValueError('profile must be normal or zero')
    if mode == 'zero':
        config['noise_enabled']=False
        config['delay_enabled']=False
    validate(config)
    return config, platform, camera


def validate(config):
    """Pure measurement-configuration check; no ROS node or filesystem needed."""
    gnss_covariance(config)
    for key in ('steer_sigma_rad','wheel_radius_relative_sigma','gyro_sigma_rad_s',
                'gyro_bias_sigma_rad_s','accel_sigma_m_s2','accel_bias_sigma_m_s2'):
        if not np.isfinite(config[key]) or config[key]<0:
            raise ValueError('invalid '+key)
    for key in ('motion_tilt_window_s','motion_tilt_interval_s','motion_tilt_extra_sigma_m_s2','motion_tilt_reject_m_s2','motion_tilt_accel_fraction','motion_tilt_max_accel_m_s2'):
        if not np.isfinite(config[key]) or config[key]<=0:
            raise ValueError('invalid '+key)
    samples=config['stationary_tilt_samples']
    if not isinstance(samples,int) or isinstance(samples,bool) or not 2<=samples<=1000:
        raise ValueError('invalid stationary_tilt_samples')
    if not isinstance(config['motion_tilt_enabled'],bool):raise ValueError('invalid motion_tilt_enabled')
    for key in ('imu_delay_s','wheel_delay_s','gnss_delay_s'):
        if min(finite(config[key],(2,)))<0: raise ValueError('invalid delay')
    if not np.isfinite(config['encoder_counts_per_revolution']) or config['encoder_counts_per_revolution']<=0 or not 0<config['gnss_frequency_hz']<=50:
        raise ValueError('invalid encoder count or GNSS frequency')
    if not np.isfinite(config['contact_normal_speed_sigma_m_s']) or config['contact_normal_speed_sigma_m_s']<=0:
        raise ValueError('invalid contact constraint uncertainty')
    return config
