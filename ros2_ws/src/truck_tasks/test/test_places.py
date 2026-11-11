import pytest

from truck_tasks.places_node import PlaceBook, normalize, resolve_waypoints


def test_normalize():
    assert normalize('  Kitchen_Door ') == 'kitchen door'


def test_book_persists(tmp_path):
    f = str(tmp_path / 'places.json')
    b = PlaceBook(f)
    b.set('Kitchen Door', 1.234, -0.5)
    b.set('couch', 2, 3)
    assert b.delete('COUCH') and not b.delete('nope')
    b2 = PlaceBook(f)
    assert b2.list() == [{'name': 'Kitchen Door', 'x': 1.23, 'y': -0.5}]
    assert b2.get('kitchen_door')['x'] == 1.23


def test_resolve_mixed_waypoints():
    places = {'kitchen door': {'name': 'Kitchen Door', 'x': 1.0, 'y': 2.0}}
    assert resolve_waypoints([[0, 0], 'kitchen_door', 'KITCHEN DOOR'], places) == \
        [(0.0, 0.0), (1.0, 2.0), (1.0, 2.0)]
    with pytest.raises(ValueError, match='unknown place'):
        resolve_waypoints(['garage'], places)
    with pytest.raises(ValueError):
        resolve_waypoints([[1]], places)
