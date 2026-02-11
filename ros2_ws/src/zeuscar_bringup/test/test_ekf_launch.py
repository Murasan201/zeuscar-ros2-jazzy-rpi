"""EKFオドメトリ・IMUフィルタ関連のテスト.

テスト対象:
- ekf_params.yaml設定ファイルの存在と内容検証
- imu_filter_params.yaml設定ファイルの存在と内容検証
- odometry.launch.pyの構造検証
- sensors.launch.pyのmadgwickノード検証
- zeuscar.launch.pyのuse_ekf引数検証
- package.xmlの依存関係確認（robot_localization, imu_filter_madgwick）

STORY-011: オドメトリ統合・SLAM動作確認
"""
import importlib.util
import os
import xml.etree.ElementTree as ET

import pytest
import yaml


# ============================================================
# ヘルパー関数
# ============================================================

def _get_bringup_base_dir() -> str:
    """zeuscar_bringupパッケージのルートディレクトリを返す."""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _get_launch_file_path(filename: str) -> str:
    """bringupパッケージのlaunchファイルパスを返す."""
    return os.path.join(_get_bringup_base_dir(), 'launch', filename)


def _get_config_file_path(filename: str) -> str:
    """bringupパッケージのconfigファイルパスを返す."""
    return os.path.join(_get_bringup_base_dir(), 'config', filename)


def _load_launch_module(filepath: str):
    """launchファイルをPythonモジュールとして読み込む."""
    module_name = os.path.splitext(os.path.basename(filepath))[0]
    spec = importlib.util.spec_from_file_location(module_name, filepath)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _get_launch_arguments(launch_description) -> dict:
    """LaunchDescriptionからDeclareLaunchArgument一覧を取得する."""
    from launch.actions import DeclareLaunchArgument

    args = {}
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            args[entity.name] = entity
    return args


def _get_default_value_text(arg) -> str:
    """DeclareLaunchArgumentのdefault_valueから文字列を取得する."""
    return ''.join(sub.text for sub in arg.default_value)


# ============================================================
# ekf_params.yaml 設定ファイルテスト
# ============================================================

class TestEkfParamsFile:
    """EKF設定ファイルの存在と内容を検証する."""

    @pytest.fixture()
    def ekf_params_path(self):
        """ekf_params.yamlのパスを返す."""
        return _get_config_file_path('ekf_params.yaml')

    @pytest.fixture()
    def ekf_params(self, ekf_params_path):
        """ekf_params.yamlを読み込んで返す."""
        if not os.path.isfile(ekf_params_path):
            pytest.skip('ekf_params.yaml が未作成')
        with open(ekf_params_path) as f:
            return yaml.safe_load(f)

    def test_ekf_params_file_exists(self, ekf_params_path):
        """ekf_params.yamlが存在する."""
        assert os.path.isfile(ekf_params_path), (
            f'ekf_params.yaml が見つかりません: {ekf_params_path}'
        )

    def test_has_ekf_filter_node_section(self, ekf_params):
        """ekf_filter_nodeセクションが存在する."""
        assert 'ekf_filter_node' in ekf_params

    def test_has_ros_parameters(self, ekf_params):
        """ros__parametersが存在する."""
        assert 'ros__parameters' in ekf_params['ekf_filter_node']

    def test_odom_frame_is_odom(self, ekf_params):
        """odom_frameがodomに設定されている."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        assert params.get('odom_frame') == 'odom'

    def test_base_link_frame_is_base_footprint(self, ekf_params):
        """base_link_frameがbase_footprintに設定されている."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        assert params.get('base_link_frame') == 'base_footprint'

    def test_world_frame_is_odom(self, ekf_params):
        """world_frameがodomに設定されている."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        assert params.get('world_frame') == 'odom'

    def test_imu0_topic_configured(self, ekf_params):
        """imu0が/imu/data（フィルタ済み）に設定されている."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        assert params.get('imu0') == '/imu/data'

    def test_imu0_config_has_15_elements(self, ekf_params):
        """imu0_configが15要素のリストである."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        config = params.get('imu0_config')
        assert isinstance(config, list), 'imu0_configがリストではありません'
        assert len(config) == 15, (
            f'imu0_configは15要素である必要があります（現在: {len(config)}）'
        )

    def test_imu0_config_angular_velocity_enabled(self, ekf_params):
        """imu0_configで角速度（インデックス9-11）が有効になっている."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        config = params.get('imu0_config', [])
        # インデックス 9, 10, 11 が角速度 (wx, wy, wz)
        assert config[9] is True, 'wx（角速度X）が無効になっています'
        assert config[10] is True, 'wy（角速度Y）が無効になっています'
        assert config[11] is True, 'wz（角速度Z）が無効になっています'

    def test_imu0_config_orientation_roll_pitch_enabled(self, ekf_params):
        """imu0_configで姿勢のroll,pitch（インデックス3,4）が有効になっている."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        config = params.get('imu0_config', [])
        # インデックス 3, 4 が姿勢 (roll, pitch)
        assert config[3] is True, 'roll（姿勢X）が無効になっています'
        assert config[4] is True, 'pitch（姿勢Y）が無効になっています'

    def test_imu0_config_orientation_yaw_disabled(self, ekf_params):
        """imu0_configで姿勢のyaw（インデックス5）が無効になっている（磁力計なし）."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        config = params.get('imu0_config', [])
        # インデックス 5 が姿勢 (yaw) - 磁力計がないため無効
        assert config[5] is False, 'yaw（姿勢Z）が有効になっています（磁力計なしのため無効にすべき）'

    def test_imu0_config_position_disabled(self, ekf_params):
        """imu0_configで位置（インデックス0-2）が無効になっている."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        config = params.get('imu0_config', [])
        assert config[0] is False, 'x位置が有効になっています'
        assert config[1] is False, 'y位置が有効になっています'
        assert config[2] is False, 'z位置が有効になっています'

    def test_imu0_config_acceleration_disabled(self, ekf_params):
        """imu0_configで加速度（インデックス12-14）が無効になっている."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        config = params.get('imu0_config', [])
        assert config[12] is False, 'ax（加速度X）が有効になっています'
        assert config[13] is False, 'ay（加速度Y）が有効になっています'
        assert config[14] is False, 'az（加速度Z）が有効になっています'

    def test_gravitational_acceleration_removal(self, ekf_params):
        """重力加速度の除去が有効になっている."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        assert params.get('imu0_remove_gravitational_acceleration') is True

    def test_frequency_set(self, ekf_params):
        """frequencyが設定されている."""
        params = ekf_params['ekf_filter_node']['ros__parameters']
        freq = params.get('frequency')
        assert freq is not None, 'frequencyが設定されていません'
        assert freq > 0, f'frequencyが正の値ではありません: {freq}'


# ============================================================
# odometry.launch.py テスト
# ============================================================

class TestOdometryLaunchFile:
    """odometry.launch.pyの構造を検証する."""

    @pytest.fixture()
    def launch_path(self):
        """odometry.launch.pyのパスを返す."""
        return _get_launch_file_path('odometry.launch.py')

    @pytest.fixture()
    def launch_desc(self, launch_path):
        """odometry.launch.pyのLaunchDescriptionを生成."""
        if not os.path.isfile(launch_path):
            pytest.skip('odometry.launch.py が未作成')
        module = _load_launch_module(launch_path)
        return module.generate_launch_description()

    def test_odometry_launch_exists(self, launch_path):
        """odometry.launch.pyが存在する."""
        assert os.path.isfile(launch_path), (
            f'odometry.launch.py が見つかりません: {launch_path}'
        )

    def test_odometry_launch_importable(self, launch_path):
        """odometry.launch.pyがimport可能."""
        if not os.path.isfile(launch_path):
            pytest.skip('odometry.launch.py が未作成')
        module = _load_launch_module(launch_path)
        assert hasattr(module, 'generate_launch_description')

    def test_has_use_sim_time_arg(self, launch_desc):
        """use_sim_time引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_sim_time' in args

    def test_use_sim_time_default_false(self, launch_desc):
        """use_sim_timeのデフォルト値はfalse."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_sim_time']) == 'false'

    def test_contains_ekf_node(self, launch_desc):
        """LaunchDescriptionにEKFノード（robot_localization）が含まれている."""
        from launch_ros.actions import Node

        nodes = [
            entity for entity in launch_desc.entities
            if isinstance(entity, Node)
        ]
        ekf_nodes = [
            n for n in nodes
            if 'robot_localization' in str(n._Node__package)
        ]
        assert len(ekf_nodes) >= 1, (
            'robot_localizationパッケージのノードが見つかりません'
        )


# ============================================================
# zeuscar.launch.py use_ekf 引数テスト
# ============================================================

class TestZeuscarEkfArgument:
    """zeuscar.launch.pyのuse_ekf引数を検証する."""

    @pytest.fixture()
    def launch_desc(self):
        """zeuscar.launch.pyのLaunchDescriptionを生成."""
        path = _get_launch_file_path('zeuscar.launch.py')
        if not os.path.isfile(path):
            pytest.skip('zeuscar.launch.py が未作成')
        module = _load_launch_module(path)
        return module.generate_launch_description()

    def test_has_use_ekf_arg(self, launch_desc):
        """use_ekf引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_ekf' in args, (
            'zeuscar.launch.py に use_ekf 引数が定義されていません'
        )

    def test_use_ekf_default_true(self, launch_desc):
        """use_ekfのデフォルト値はtrue."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_ekf']) == 'true', (
            'use_ekf のデフォルト値が true ではありません'
        )


# ============================================================
# imu_filter_params.yaml 設定ファイルテスト
# ============================================================

class TestImuFilterParamsFile:
    """IMUフィルタ設定ファイルの存在と内容を検証する."""

    @pytest.fixture()
    def imu_filter_params_path(self):
        """imu_filter_params.yamlのパスを返す."""
        return _get_config_file_path('imu_filter_params.yaml')

    @pytest.fixture()
    def imu_filter_params(self, imu_filter_params_path):
        """imu_filter_params.yamlを読み込んで返す."""
        if not os.path.isfile(imu_filter_params_path):
            pytest.skip('imu_filter_params.yaml が未作成')
        with open(imu_filter_params_path) as f:
            return yaml.safe_load(f)

    def test_imu_filter_params_file_exists(self, imu_filter_params_path):
        """imu_filter_params.yamlが存在する."""
        assert os.path.isfile(imu_filter_params_path), (
            f'imu_filter_params.yaml が見つかりません: {imu_filter_params_path}'
        )

    def test_has_imu_filter_node_section(self, imu_filter_params):
        """imu_filter_madgwick_nodeセクションが存在する."""
        assert 'imu_filter_madgwick_node' in imu_filter_params, (
            'imu_filter_madgwick_node セクションが見つかりません。'
            f'トップレベルキー: {list(imu_filter_params.keys())}'
        )

    def test_use_mag_is_false(self, imu_filter_params):
        """use_magがfalseに設定されている（磁力計なし）."""
        params = imu_filter_params['imu_filter_madgwick_node']['ros__parameters']
        assert params.get('use_mag') is False, (
            'use_mag が false ではありません（磁力計なしのため false にすべき）'
        )

    def test_publish_tf_is_false(self, imu_filter_params):
        """publish_tfがfalseに設定されている（TFはEKFが担当）."""
        params = imu_filter_params['imu_filter_madgwick_node']['ros__parameters']
        assert params.get('publish_tf') is False, (
            'publish_tf が false ではありません（TF配信はEKFが担当するため false にすべき）'
        )

    def test_world_frame_is_enu(self, imu_filter_params):
        """world_frameがenuに設定されている."""
        params = imu_filter_params['imu_filter_madgwick_node']['ros__parameters']
        assert params.get('world_frame') == 'enu', (
            f'world_frame が "enu" ではありません: {params.get("world_frame")}'
        )


# ============================================================
# sensors.launch.py Madgwick ノードテスト
# ============================================================

class TestSensorsLaunchMadgwick:
    """sensors.launch.pyにmadgwickノードが含まれることを検証する."""

    @pytest.fixture()
    def sensors_launch_path(self):
        """sensors.launch.pyのパスを返す."""
        return _get_launch_file_path('sensors.launch.py')

    @pytest.fixture()
    def sensors_launch_desc(self, sensors_launch_path):
        """sensors.launch.pyのLaunchDescriptionを生成."""
        if not os.path.isfile(sensors_launch_path):
            pytest.skip('sensors.launch.py が未作成')
        module = _load_launch_module(sensors_launch_path)
        return module.generate_launch_description()

    def test_sensors_launch_has_madgwick_node(self, sensors_launch_desc):
        """LaunchDescription内にimu_filter_madgwickパッケージのNodeが含まれている."""
        from launch_ros.actions import Node

        nodes = [
            entity for entity in sensors_launch_desc.entities
            if isinstance(entity, Node)
        ]
        madgwick_nodes = [
            n for n in nodes
            if 'imu_filter_madgwick' in str(n._Node__package)
        ]
        assert len(madgwick_nodes) >= 1, (
            'sensors.launch.py に imu_filter_madgwick パッケージのノードが見つかりません。'
            f'検出されたNode数: {len(nodes)}'
        )


# ============================================================
# package.xml 依存関係テスト
# ============================================================

class TestPackageXmlDependency:
    """package.xmlにrobot_localizationの依存が宣言されている."""

    @pytest.fixture()
    def package_xml_path(self):
        """package.xmlのパスを返す."""
        return os.path.join(_get_bringup_base_dir(), 'package.xml')

    def test_has_robot_localization_dependency(self, package_xml_path):
        """package.xmlにrobot_localizationのexec_dependが存在する."""
        tree = ET.parse(package_xml_path)
        root = tree.getroot()
        exec_deps = [dep.text for dep in root.findall('exec_depend')]
        assert 'robot_localization' in exec_deps, (
            'package.xml に robot_localization の exec_depend がありません。'
            f'現在の exec_depend: {exec_deps}'
        )

    def test_has_imu_filter_madgwick_dependency(self, package_xml_path):
        """package.xmlにimu_filter_madgwickのexec_dependが存在する."""
        tree = ET.parse(package_xml_path)
        root = tree.getroot()
        exec_deps = [dep.text for dep in root.findall('exec_depend')]
        assert 'imu_filter_madgwick' in exec_deps, (
            'package.xml に imu_filter_madgwick の exec_depend がありません。'
            f'現在の exec_depend: {exec_deps}'
        )
