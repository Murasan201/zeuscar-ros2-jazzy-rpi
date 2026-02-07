"""URDFにimu_linkが定義されているかのテスト.

テスト対象:
- URDFファイル内にimu_linkリンクが存在すること
- base_link → imu_linkのジョイントが定義されていること
- imu_linkの親がbase_linkであること
- TFツリーの構造が設計仕様に合致すること

設計仕様書参照:
- docs/operations/specs/STORY-014-015_bringup_design.md セクション2.2
"""
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest


# ============================================================
# ヘルパー関数
# ============================================================

def _get_urdf_path() -> str:
    """URDFファイルのパスを返す."""
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    # zeuscar_bringup と同階層の zeuscar_description を参照
    description_dir = os.path.join(
        os.path.dirname(base_dir), 'zeuscar_description'
    )
    return os.path.join(description_dir, 'urdf', 'zeuscar.urdf.xacro')


def _process_xacro(xacro_path: str) -> str:
    """xacroファイルを処理してURDF XMLを返す.

    Args:
        xacro_path: xacroファイルの絶対パス

    Returns:
        処理済みのURDF XML文字列
    """
    result = subprocess.run(
        ['xacro', xacro_path],
        capture_output=True,
        text=True,
        timeout=10,
    )
    if result.returncode != 0:
        pytest.fail(f'xacro処理失敗: {result.stderr}')
    return result.stdout


def _parse_urdf(xml_string: str) -> ET.Element:
    """URDF XML文字列をパースする."""
    return ET.fromstring(xml_string)


def _get_all_links(root: ET.Element) -> list[str]:
    """URDFから全リンク名を取得する."""
    return [link.get('name') for link in root.findall('link')]


def _get_all_joints(root: ET.Element) -> list[dict]:
    """URDFから全ジョイント情報を取得する.

    Returns:
        [{'name': str, 'type': str, 'parent': str, 'child': str}, ...]
    """
    joints = []
    for joint in root.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        joints.append({
            'name': joint.get('name'),
            'type': joint.get('type'),
            'parent': parent.get('link') if parent is not None else None,
            'child': child.get('link') if child is not None else None,
        })
    return joints


# ============================================================
# xacroファイル検証テスト
# ============================================================

class TestUrdfXacroValid:
    """URDFファイルがxacro処理可能であることを確認."""

    def test_xacro_file_exists(self):
        """xacroファイルが存在する."""
        path = _get_urdf_path()
        assert os.path.isfile(path), f'URDFファイルが見つかりません: {path}'

    def test_xacro_processes_without_error(self):
        """xacro処理がエラーなく完了する."""
        path = _get_urdf_path()
        if not os.path.isfile(path):
            pytest.skip('URDFファイルが未作成')
        xml = _process_xacro(path)
        assert len(xml) > 0, 'xacro出力が空です'

    def test_urdf_is_valid_xml(self):
        """xacro出力が有効なXMLである."""
        path = _get_urdf_path()
        if not os.path.isfile(path):
            pytest.skip('URDFファイルが未作成')
        xml = _process_xacro(path)
        root = _parse_urdf(xml)
        assert root.tag == 'robot'


# ============================================================
# imu_linkリンク定義テスト
# ============================================================

class TestImuLinkDefinition:
    """URDFにimu_linkが正しく定義されていることを確認."""

    @pytest.fixture()
    def urdf_root(self):
        """xacro処理済みURDFのルート要素."""
        path = _get_urdf_path()
        if not os.path.isfile(path):
            pytest.skip('URDFファイルが未作成')
        xml = _process_xacro(path)
        return _parse_urdf(xml)

    def test_imu_link_exists(self, urdf_root):
        """imu_linkリンクが定義されている."""
        links = _get_all_links(urdf_root)
        assert 'imu_link' in links, (
            f'imu_linkが見つかりません。定義済みリンク: {links}'
        )

    def test_imu_link_has_joint(self, urdf_root):
        """imu_linkへのジョイントが定義されている."""
        joints = _get_all_joints(urdf_root)
        imu_joints = [j for j in joints if j['child'] == 'imu_link']
        assert len(imu_joints) > 0, (
            'imu_linkを子とするジョイントが見つかりません'
        )

    def test_imu_link_parent_is_base_link(self, urdf_root):
        """imu_linkの親リンクがbase_linkである."""
        joints = _get_all_joints(urdf_root)
        imu_joints = [j for j in joints if j['child'] == 'imu_link']
        if not imu_joints:
            pytest.fail('imu_linkを子とするジョイントが見つかりません')
        assert imu_joints[0]['parent'] == 'base_link', (
            f'imu_linkの親が base_link ではなく '
            f'{imu_joints[0]["parent"]} です'
        )

    def test_imu_joint_is_fixed(self, urdf_root):
        """imu_linkのジョイントタイプがfixedである."""
        joints = _get_all_joints(urdf_root)
        imu_joints = [j for j in joints if j['child'] == 'imu_link']
        if not imu_joints:
            pytest.fail('imu_linkを子とするジョイントが見つかりません')
        assert imu_joints[0]['type'] == 'fixed', (
            f'ジョイントタイプが fixed ではなく '
            f'{imu_joints[0]["type"]} です'
        )


# ============================================================
# TFツリー構造テスト
# ============================================================

class TestTfTreeStructure:
    """TFツリー構造が設計仕様に合致することを確認.

    設計目標のTFツリー:
        base_footprint → base_link → laser_frame
                                   → imu_link
    """

    @pytest.fixture()
    def urdf_root(self):
        """xacro処理済みURDFのルート要素."""
        path = _get_urdf_path()
        if not os.path.isfile(path):
            pytest.skip('URDFファイルが未作成')
        xml = _process_xacro(path)
        return _parse_urdf(xml)

    def _build_tf_tree(self, root: ET.Element) -> dict:
        """ジョイント情報からTFツリー（親→子のリスト）を構築する.

        Returns:
            {親リンク名: [子リンク名, ...], ...}
        """
        tree = {}
        joints = _get_all_joints(root)
        for joint in joints:
            parent = joint['parent']
            child = joint['child']
            if parent not in tree:
                tree[parent] = []
            tree[parent].append(child)
        return tree

    def test_base_footprint_to_base_link(self, urdf_root):
        """base_footprint → base_link の接続がある."""
        tree = self._build_tf_tree(urdf_root)
        assert 'base_footprint' in tree, (
            'base_footprintを親とするジョイントがありません'
        )
        assert 'base_link' in tree['base_footprint'], (
            'base_footprint → base_link の接続がありません'
        )

    def test_base_link_to_laser_frame(self, urdf_root):
        """base_link → laser_frame の接続がある."""
        tree = self._build_tf_tree(urdf_root)
        assert 'base_link' in tree, (
            'base_linkを親とするジョイントがありません'
        )
        assert 'laser_frame' in tree['base_link'], (
            'base_link → laser_frame の接続がありません'
        )

    def test_base_link_to_imu_link(self, urdf_root):
        """base_link → imu_link の接続がある."""
        tree = self._build_tf_tree(urdf_root)
        assert 'base_link' in tree, (
            'base_linkを親とするジョイントがありません'
        )
        assert 'imu_link' in tree['base_link'], (
            'base_link → imu_link の接続がありません'
        )

    def test_base_link_has_two_children(self, urdf_root):
        """base_linkはlaser_frameとimu_linkの2つの子を持つ."""
        tree = self._build_tf_tree(urdf_root)
        assert 'base_link' in tree
        children = tree['base_link']
        assert 'laser_frame' in children, (
            'base_linkの子にlaser_frameがありません'
        )
        assert 'imu_link' in children, (
            'base_linkの子にimu_linkがありません'
        )

    def test_expected_link_count(self, urdf_root):
        """URDFに期待されるリンク数が存在する.

        期待: base_footprint, base_link, laser_frame, imu_link の4つ
        """
        links = _get_all_links(urdf_root)
        expected = {'base_footprint', 'base_link', 'laser_frame', 'imu_link'}
        assert expected.issubset(set(links)), (
            f'不足リンク: {expected - set(links)}'
        )
