from __future__ import annotations

import pytest

from sdk import ArticulatedObject, Box, Cylinder, Material, ValidationError


def test_part_get_visual_returns_named_visual() -> None:
    model = ArticulatedObject(name="visual_lookup")
    part = model.part("body")
    visual = part.visual(Box((0.1, 0.1, 0.1)), name="shell")

    assert part.get_visual("shell") is visual


def test_part_get_visual_rejects_missing_name() -> None:
    model = ArticulatedObject(name="visual_lookup")
    part = model.part("body")
    part.visual(Box((0.1, 0.1, 0.1)), name="shell")

    with pytest.raises(Exception, match="Unknown visual"):
        part.get_visual("missing")


def test_cylinder_accepts_height_alias() -> None:
    cylinder = Cylinder(radius=0.1, height=0.4)

    assert cylinder.radius == 0.1
    assert cylinder.length == 0.4
    assert cylinder.height == 0.4


def test_cylinder_rejects_length_and_height_together() -> None:
    with pytest.raises(TypeError, match="only one of 'length' or alias 'height'"):
        Cylinder(radius=0.1, length=0.4, height=0.4)


def test_part_visual_accepts_color_alias_as_named_material() -> None:
    model = ArticulatedObject(name="visual_color_alias")
    part = model.part("body")

    visual = part.visual(Box((0.1, 0.1, 0.1)), color="steel_blue", name="shell")

    assert visual.material == "steel_blue"


def test_part_visual_accepts_color_alias_as_rgba_tuple() -> None:
    model = ArticulatedObject(name="visual_color_alias")
    part = model.part("body")

    visual = part.visual(Box((0.1, 0.1, 0.1)), color=(0.1, 0.2, 0.3, 0.4), name="shell")

    assert isinstance(visual.material, Material)
    assert visual.material.rgba == (0.1, 0.2, 0.3, 0.4)


def test_part_visual_rejects_material_and_color_together() -> None:
    model = ArticulatedObject(name="visual_color_alias")
    part = model.part("body")

    with pytest.raises(TypeError, match="only one of 'material' or alias 'color'"):
        part.visual(Box((0.1, 0.1, 0.1)), material="steel", color="blue")


def test_material_accepts_color_alias_with_rgb_tuple() -> None:
    material = Material(name="paint", color=(0.1, 0.2, 0.3))

    assert material.rgba == (0.1, 0.2, 0.3, 1.0)


def test_material_accepts_color_alias_with_rgba_tuple() -> None:
    material = Material(name="paint", color=(0.1, 0.2, 0.3, 0.4))

    assert material.rgba == (0.1, 0.2, 0.3, 0.4)


def test_material_rejects_rgba_and_color_together() -> None:
    with pytest.raises(ValidationError, match="Material cannot set both rgba and color"):
        Material(name="paint", rgba=(0.1, 0.2, 0.3, 1.0), color=(0.2, 0.3, 0.4, 1.0))


def test_material_preserves_positional_rgba_usage() -> None:
    material = Material("paint", (0.1, 0.2, 0.3, 1.0))

    assert material.rgba == (0.1, 0.2, 0.3, 1.0)
