from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OPTICAL_AXIS = Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _add_barrel_half(part, *, side: float, material_names: dict[str, str]) -> None:
    """Add one compact porro-prism binocular half in the part's local hinge frame."""
    x = side * 0.034

    part.visual(
        Cylinder(radius=0.0150, length=0.102),
        origin=Origin(xyz=(x, 0.000, 0.000), rpy=OPTICAL_AXIS.rpy),
        material=material_names["armor"],
        name="main_tube",
    )
    part.visual(
        Cylinder(radius=0.0180, length=0.014),
        origin=Origin(xyz=(x, 0.056, 0.000), rpy=OPTICAL_AXIS.rpy),
        material=material_names["black"],
        name="objective_ring",
    )
    part.visual(
        Cylinder(radius=0.0128, length=0.0025),
        origin=Origin(xyz=(x, 0.063, 0.000), rpy=OPTICAL_AXIS.rpy),
        material=material_names["glass"],
        name="objective_glass",
    )
    part.visual(
        Cylinder(radius=0.0130, length=0.016),
        origin=Origin(xyz=(x, -0.057, 0.000), rpy=OPTICAL_AXIS.rpy),
        material=material_names["rubber"],
        name="eyecup",
    )
    part.visual(
        Cylinder(radius=0.0085, length=0.0020),
        origin=Origin(xyz=(x, -0.066, 0.000), rpy=OPTICAL_AXIS.rpy),
        material=material_names["glass_dark"],
        name="ocular_glass",
    )

    # Porro prism shoulders: compact binoculars have armored offset housings
    # rather than two plain tubes.
    part.visual(
        Box((0.032, 0.043, 0.021)),
        origin=Origin(xyz=(x, -0.020, 0.014)),
        material=material_names["armor"],
        name="rear_prism_housing",
    )
    part.visual(
        Box((0.030, 0.038, 0.017)),
        origin=Origin(xyz=(x, 0.025, 0.012)),
        material=material_names["armor"],
        name="front_prism_housing",
    )
    part.visual(
        Box((0.027, 0.006, 0.004)),
        origin=Origin(xyz=(x, 0.045, 0.018)),
        material=material_names["trim"],
        name="front_grip_band",
    )
    part.visual(
        Box((0.028, 0.006, 0.004)),
        origin=Origin(xyz=(x, -0.040, 0.021)),
        material=material_names["trim"],
        name="rear_grip_band",
    )

    bridge_x = side * 0.014
    for y, name in ((-0.043, "rear_bridge_arm"), (0.043, "front_bridge_arm")):
        part.visual(
            Box((0.023, 0.010, 0.009)),
            origin=Origin(xyz=(bridge_x, y, 0.009)),
            material=material_names["black"],
            name=name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_8x25_porro_binocular")

    model.material("matte_black", rgba=(0.015, 0.016, 0.017, 1.0))
    model.material("rubber_armor", rgba=(0.055, 0.060, 0.058, 1.0))
    model.material("soft_eyecup", rgba=(0.020, 0.021, 0.022, 1.0))
    model.material("dark_trim", rgba=(0.11, 0.115, 0.115, 1.0))
    model.material("blue_coated_glass", rgba=(0.20, 0.42, 0.58, 0.62))
    model.material("dark_lens_glass", rgba=(0.05, 0.09, 0.12, 0.72))
    model.material("satin_steel", rgba=(0.50, 0.52, 0.51, 1.0))

    material_names = {
        "black": "matte_black",
        "armor": "rubber_armor",
        "rubber": "soft_eyecup",
        "trim": "dark_trim",
        "glass": "blue_coated_glass",
        "glass_dark": "dark_lens_glass",
    }

    barrel_0 = model.part("barrel_0")
    _add_barrel_half(barrel_0, side=-1.0, material_names=material_names)
    for y, name in ((-0.043, "rear_hinge_knuckle"), (0.043, "front_hinge_knuckle")):
        barrel_0.visual(
            Cylinder(radius=0.0065, length=0.026),
            origin=Origin(xyz=(0.0, y, 0.000), rpy=OPTICAL_AXIS.rpy),
            material="satin_steel",
            name=name,
        )

    # A small yoke holds the focus wheel just above the center bridge.
    barrel_0.visual(
        Box((0.010, 0.034, 0.005)),
        origin=Origin(xyz=(0.0, -0.038, 0.0165)),
        material="matte_black",
        name="focus_yoke_base",
    )
    for y, name in ((-0.0498, "rear_focus_cheek"), (-0.0262, "front_focus_cheek")):
        barrel_0.visual(
            Box((0.011, 0.0030, 0.026)),
            origin=Origin(xyz=(0.0, y, 0.031)),
            material="matte_black",
            name=name,
        )
    barrel_0.visual(
        Cylinder(radius=0.0030, length=0.022),
        origin=Origin(xyz=(0.0, -0.043, 0.010)),
        material="satin_steel",
        name="focus_yoke_post",
    )

    barrel_1 = model.part("barrel_1")
    _add_barrel_half(barrel_1, side=1.0, material_names=material_names)
    barrel_1.visual(
        Cylinder(radius=0.0060, length=0.052),
        origin=Origin(xyz=(0.0, 0.000, 0.000), rpy=OPTICAL_AXIS.rpy),
        material="satin_steel",
        name="center_hinge_knuckle",
    )
    barrel_1.visual(
        Box((0.023, 0.018, 0.009)),
        origin=Origin(xyz=(0.014, 0.000, 0.009)),
        material="matte_black",
        name="center_bridge_arm",
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        Cylinder(radius=0.0120, length=0.020),
        origin=Origin(rpy=OPTICAL_AXIS.rpy),
        material="matte_black",
        name="wheel_core",
    )
    for index, y in enumerate((-0.008, -0.004, 0.000, 0.004, 0.008)):
        focus_wheel.visual(
            Cylinder(radius=0.0140, length=0.0014),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=OPTICAL_AXIS.rpy),
            material="dark_trim",
            name=f"raised_rib_{index}",
        )

    model.articulation(
        "central_hinge",
        ArticulationType.REVOLUTE,
        parent=barrel_0,
        child=barrel_1,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=1.0, lower=-0.24, upper=0.24),
    )
    model.articulation(
        "center_focus",
        ArticulationType.REVOLUTE,
        parent=barrel_0,
        child=focus_wheel,
        origin=Origin(xyz=(0.0, -0.038, 0.033)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=5.0, lower=-3.14, upper=3.14),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel_0 = object_model.get_part("barrel_0")
    barrel_1 = object_model.get_part("barrel_1")
    focus_wheel = object_model.get_part("focus_wheel")
    hinge = object_model.get_articulation("central_hinge")
    focus = object_model.get_articulation("center_focus")

    ctx.check(
        "binocular has revolute central hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "focus wheel has revolute joint",
        focus.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={focus.articulation_type}",
    )
    ctx.expect_gap(
        barrel_1,
        barrel_0,
        axis="x",
        positive_elem="main_tube",
        negative_elem="main_tube",
        min_gap=0.035,
        max_gap=0.041,
        name="main optical tubes are symmetrically separated",
    )
    ctx.expect_gap(
        focus_wheel,
        barrel_0,
        axis="z",
        positive_elem="wheel_core",
        negative_elem="focus_yoke_base",
        min_gap=0.0003,
        max_gap=0.0025,
        name="focus wheel sits just above its yoke",
    )
    ctx.expect_gap(
        barrel_0,
        focus_wheel,
        axis="y",
        positive_elem="front_focus_cheek",
        negative_elem="wheel_core",
        min_gap=0.0,
        max_gap=0.0015,
        name="front focus cheek captures wheel end",
    )
    ctx.expect_gap(
        focus_wheel,
        barrel_0,
        axis="y",
        positive_elem="wheel_core",
        negative_elem="rear_focus_cheek",
        min_gap=0.0,
        max_gap=0.0015,
        name="rear focus cheek captures wheel end",
    )

    rest_aabb = ctx.part_element_world_aabb(barrel_1, elem="main_tube")
    with ctx.pose({hinge: 0.18}):
        folded_aabb = ctx.part_element_world_aabb(barrel_1, elem="main_tube")
    ctx.check(
        "central hinge changes barrel angle",
        rest_aabb is not None
        and folded_aabb is not None
        and abs(folded_aabb[0][2] - rest_aabb[0][2]) > 0.003,
        details=f"rest={rest_aabb}, folded={folded_aabb}",
    )

    rest_focus_position = ctx.part_world_position(focus_wheel)
    with ctx.pose({focus: 1.0}):
        turned_focus_position = ctx.part_world_position(focus_wheel)
    ctx.check(
        "focus wheel rotates in place",
        rest_focus_position is not None
        and turned_focus_position is not None
        and max(abs(a - b) for a, b in zip(rest_focus_position, turned_focus_position)) < 1e-6,
        details=f"rest={rest_focus_position}, turned={turned_focus_position}",
    )

    return ctx.report()


object_model = build_object_model()
