"""統合bringup launchファイルのテスト.

テスト対象:
- launchファイルの存在確認（robot_base, sensors, zeuscar）
- Pythonモジュールとしての構文検証
- generate_launch_description関数の存在確認
- Launch Argumentsのデフォルト値検証
- 条件付き起動フラグの検証（use_lidar, use_imu, use_motor, use_slam）
"""
import importlib.util
import os

import pytest


# ============================================================
# ヘルパー関数
# ============================================================

def _get_launch_file_path(filename: str) -> str:
    """bringupパッケージのlaunchファイルパスを返す."""
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_dir, 'launch', filename)


def _load_launch_module(filepath: str):
    """launchファイルをPythonモジュールとして読み込む.

    Args:
        filepath: launchファイルの絶対パス

    Returns:
        読み込まれたモジュールオブジェクト
    """
    module_name = os.path.splitext(os.path.basename(filepath))[0]
    spec = importlib.util.spec_from_file_location(module_name, filepath)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _get_launch_arguments(launch_description) -> dict:
    """LaunchDescriptionからDeclareLaunchArgument一覧を取得する.

    Returns:
        {引数名: DeclareLaunchArgumentオブジェクト} の辞書
    """
    from launch.actions import DeclareLaunchArgument

    args = {}
    for entity in launch_description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            args[entity.name] = entity
    return args


def _get_default_value_text(arg) -> str:
    """DeclareLaunchArgumentのdefault_valueから文字列を取得する.

    default_valueはTextSubstitutionオブジェクトのリストとして保持されるため、
    各要素の.textプロパティを結合して返す。
    """
    return ''.join(sub.text for sub in arg.default_value)


# ============================================================
# launchファイル存在確認テスト
# ============================================================

class TestLaunchFileExistence:
    """launchファイルの存在を確認するテスト."""

    def test_robot_base_launch_exists(self):
        """robot_base.launch.pyが存在する."""
        path = _get_launch_file_path('robot_base.launch.py')
        assert os.path.isfile(path), (
            f'robot_base.launch.py が見つかりません: {path}'
        )

    def test_sensors_launch_exists(self):
        """sensors.launch.pyが存在する."""
        path = _get_launch_file_path('sensors.launch.py')
        assert os.path.isfile(path), (
            f'sensors.launch.py が見つかりません: {path}'
        )

    def test_zeuscar_launch_exists(self):
        """zeuscar.launch.pyが存在する."""
        path = _get_launch_file_path('zeuscar.launch.py')
        assert os.path.isfile(path), (
            f'zeuscar.launch.py が見つかりません: {path}'
        )


# ============================================================
# launchファイル構文テスト
# ============================================================

class TestLaunchFileSyntax:
    """launchファイルがPythonモジュールとして正常に読み込める."""

    def test_robot_base_launch_importable(self):
        """robot_base.launch.pyがimport可能."""
        path = _get_launch_file_path('robot_base.launch.py')
        if not os.path.isfile(path):
            pytest.skip('robot_base.launch.py が未作成')
        module = _load_launch_module(path)
        assert hasattr(module, 'generate_launch_description')

    def test_sensors_launch_importable(self):
        """sensors.launch.pyがimport可能."""
        path = _get_launch_file_path('sensors.launch.py')
        if not os.path.isfile(path):
            pytest.skip('sensors.launch.py が未作成')
        module = _load_launch_module(path)
        assert hasattr(module, 'generate_launch_description')

    def test_zeuscar_launch_importable(self):
        """zeuscar.launch.pyがimport可能."""
        path = _get_launch_file_path('zeuscar.launch.py')
        if not os.path.isfile(path):
            pytest.skip('zeuscar.launch.py が未作成')
        module = _load_launch_module(path)
        assert hasattr(module, 'generate_launch_description')


# ============================================================
# robot_base.launch.py Launch Arguments テスト
# ============================================================

class TestRobotBaseLaunchArguments:
    """robot_base.launch.pyのLaunch Arguments検証."""

    @pytest.fixture()
    def launch_desc(self):
        """robot_base.launch.pyのLaunchDescriptionを生成."""
        path = _get_launch_file_path('robot_base.launch.py')
        if not os.path.isfile(path):
            pytest.skip('robot_base.launch.py が未作成')
        module = _load_launch_module(path)
        return module.generate_launch_description()

    def test_has_use_sim_time_arg(self, launch_desc):
        """use_sim_time引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_sim_time' in args

    def test_use_sim_time_default_false(self, launch_desc):
        """use_sim_timeのデフォルト値はfalse."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_sim_time']) == 'false'

    def test_has_use_motor_arg(self, launch_desc):
        """use_motor引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_motor' in args

    def test_use_motor_default_true(self, launch_desc):
        """use_motorのデフォルト値はtrue."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_motor']) == 'true'

    def test_has_serial_port_arg(self, launch_desc):
        """serial_port引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'serial_port' in args

    def test_serial_port_default_value(self, launch_desc):
        """serial_portのデフォルト値は/dev/ttyACM0."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['serial_port']) == '/dev/ttyACM0'

    def test_has_baud_rate_arg(self, launch_desc):
        """baud_rate引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'baud_rate' in args

    def test_baud_rate_default_value(self, launch_desc):
        """baud_rateのデフォルト値は9600."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['baud_rate']) == '9600'


# ============================================================
# sensors.launch.py Launch Arguments テスト
# ============================================================

class TestSensorsLaunchArguments:
    """sensors.launch.pyのLaunch Arguments検証."""

    @pytest.fixture()
    def launch_desc(self):
        """sensors.launch.pyのLaunchDescriptionを生成."""
        path = _get_launch_file_path('sensors.launch.py')
        if not os.path.isfile(path):
            pytest.skip('sensors.launch.py が未作成')
        module = _load_launch_module(path)
        return module.generate_launch_description()

    def test_has_use_lidar_arg(self, launch_desc):
        """use_lidar引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_lidar' in args

    def test_use_lidar_default_true(self, launch_desc):
        """use_lidarのデフォルト値はtrue."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_lidar']) == 'true'

    def test_has_use_imu_arg(self, launch_desc):
        """use_imu引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_imu' in args

    def test_use_imu_default_true(self, launch_desc):
        """use_imuのデフォルト値はtrue."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_imu']) == 'true'

    def test_has_serial_port_lidar_arg(self, launch_desc):
        """serial_port_lidar引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'serial_port_lidar' in args

    def test_serial_port_lidar_default_value(self, launch_desc):
        """serial_port_lidarのデフォルト値は/dev/rplidar."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['serial_port_lidar']) == '/dev/rplidar'


# ============================================================
# zeuscar.launch.py Launch Arguments テスト
# ============================================================

class TestZeuscarLaunchArguments:
    """zeuscar.launch.pyのLaunch Arguments検証."""

    @pytest.fixture()
    def launch_desc(self):
        """zeuscar.launch.pyのLaunchDescriptionを生成."""
        path = _get_launch_file_path('zeuscar.launch.py')
        if not os.path.isfile(path):
            pytest.skip('zeuscar.launch.py が未作成')
        module = _load_launch_module(path)
        return module.generate_launch_description()

    def test_has_use_sim_time_arg(self, launch_desc):
        """use_sim_time引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_sim_time' in args

    def test_use_sim_time_default_false(self, launch_desc):
        """use_sim_timeのデフォルト値はfalse."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_sim_time']) == 'false'

    def test_has_use_lidar_arg(self, launch_desc):
        """use_lidar引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_lidar' in args

    def test_use_lidar_default_true(self, launch_desc):
        """use_lidarのデフォルト値はtrue."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_lidar']) == 'true'

    def test_has_use_imu_arg(self, launch_desc):
        """use_imu引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_imu' in args

    def test_use_imu_default_true(self, launch_desc):
        """use_imuのデフォルト値はtrue."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_imu']) == 'true'

    def test_has_use_motor_arg(self, launch_desc):
        """use_motor引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_motor' in args

    def test_use_motor_default_true(self, launch_desc):
        """use_motorのデフォルト値はtrue."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_motor']) == 'true'

    def test_has_use_slam_arg(self, launch_desc):
        """use_slam引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_slam' in args

    def test_use_slam_default_false(self, launch_desc):
        """use_slamのデフォルト値はfalse."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_slam']) == 'false'

    def test_has_use_rviz_arg(self, launch_desc):
        """use_rviz引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'use_rviz' in args

    def test_use_rviz_default_false(self, launch_desc):
        """use_rvizのデフォルト値はfalse."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['use_rviz']) == 'false'

    def test_has_serial_port_motor_arg(self, launch_desc):
        """serial_port_motor引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'serial_port_motor' in args

    def test_serial_port_motor_default_value(self, launch_desc):
        """serial_port_motorのデフォルト値は/dev/ttyACM0."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['serial_port_motor']) == '/dev/ttyACM0'

    def test_has_serial_port_lidar_arg(self, launch_desc):
        """serial_port_lidar引数が定義されている."""
        args = _get_launch_arguments(launch_desc)
        assert 'serial_port_lidar' in args

    def test_serial_port_lidar_default_value(self, launch_desc):
        """serial_port_lidarのデフォルト値は/dev/rplidar."""
        args = _get_launch_arguments(launch_desc)
        assert _get_default_value_text(args['serial_port_lidar']) == '/dev/rplidar'


# ============================================================
# 条件付き起動テスト
# ============================================================

class TestConditionalLaunch:
    """条件付き起動の検証.

    LaunchDescriptionに含まれるIncludeLaunchDescriptionやNodeアクションが
    IfConditionで適切にフラグ制御されているかをテストする。
    """

    @pytest.fixture()
    def zeuscar_launch_desc(self):
        """zeuscar.launch.pyのLaunchDescriptionを生成."""
        path = _get_launch_file_path('zeuscar.launch.py')
        if not os.path.isfile(path):
            pytest.skip('zeuscar.launch.py が未作成')
        module = _load_launch_module(path)
        return module.generate_launch_description()

    def _count_conditional_entities(self, launch_desc) -> int:
        """conditionを持つエンティティの数を数える."""
        count = 0
        for entity in launch_desc.entities:
            if hasattr(entity, 'condition') and entity.condition is not None:
                count += 1
        return count

    def test_has_conditional_entities(self, zeuscar_launch_desc):
        """条件付きエンティティが存在する（use_*フラグによる制御）."""
        count = self._count_conditional_entities(zeuscar_launch_desc)
        # use_lidar, use_imu, use_motor, use_slam, use_rviz の少なくとも一部
        assert count >= 2, (
            f'条件付きエンティティが {count} 個しかない（2個以上必要）'
        )

    @pytest.fixture()
    def sensors_launch_desc(self):
        """sensors.launch.pyのLaunchDescriptionを生成."""
        path = _get_launch_file_path('sensors.launch.py')
        if not os.path.isfile(path):
            pytest.skip('sensors.launch.py が未作成')
        module = _load_launch_module(path)
        return module.generate_launch_description()

    def test_sensors_has_conditional_entities(self, sensors_launch_desc):
        """sensors.launch.pyに条件付きエンティティが存在する."""
        count = self._count_conditional_entities(sensors_launch_desc)
        # use_lidar, use_imu の少なくとも一部
        assert count >= 1, (
            f'条件付きエンティティが {count} 個しかない（1個以上必要）'
        )

    @pytest.fixture()
    def robot_base_launch_desc(self):
        """robot_base.launch.pyのLaunchDescriptionを生成."""
        path = _get_launch_file_path('robot_base.launch.py')
        if not os.path.isfile(path):
            pytest.skip('robot_base.launch.py が未作成')
        module = _load_launch_module(path)
        return module.generate_launch_description()

    def test_robot_base_has_conditional_motor(self, robot_base_launch_desc):
        """robot_base.launch.pyにモーターの条件付き起動がある."""
        count = self._count_conditional_entities(robot_base_launch_desc)
        assert count >= 1, (
            f'条件付きエンティティが {count} 個しかない（1個以上必要）'
        )
