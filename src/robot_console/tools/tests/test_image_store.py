"""ImageStore の単体テスト。"""

from PIL import Image

from robot_console.core.image_store import ImageStore


def test_get_returns_none_for_unknown_key():
    store = ImageStore()
    assert store.get('unknown') is None


def test_get_returns_none_for_none_key():
    store = ImageStore()
    store.set('panel', Image.new('RGB', (1, 1)))
    assert store.get(None) is None


def test_set_and_get_round_trip():
    store = ImageStore()
    image = Image.new('RGB', (4, 4), color='red')

    store.set('route_map', image)

    assert store.get('route_map') is image


def test_set_overwrites_previous_image():
    store = ImageStore()
    first = Image.new('RGB', (1, 1), color='red')
    second = Image.new('RGB', (1, 1), color='blue')

    store.set('panel', first)
    store.set('panel', second)

    assert store.get('panel') is second


def test_clear_removes_stored_image():
    store = ImageStore()
    store.set('panel', Image.new('RGB', (1, 1)))

    store.clear('panel')

    assert store.get('panel') is None


def test_clear_missing_key_does_not_raise():
    store = ImageStore()
    store.clear('unknown')
