from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


TUBE_RADIUS = 0.011
HINGE_RADIUS = 0.014
WIDTH = 0.76
HALF_WIDTH = WIDTH / 2.0
CENTER_TOP_Z = 1.12
CENTER_FOOT_Z = 0.055


def _cylinder_origin_between(
    p0: tuple[float, float, float], p1: tuple[float, float, float]
) -> tuple[Origin, float]:
    """Return an Origin/length for a Cylinder whose local +Z runs p0 -> p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("zero-length cylinder")

    yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-12 else 0.0
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    center = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _tube(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
    name: str,
) -> None:
    origin, length = _cylinder_origin_between(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _node(
    part,
    pos: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
    name: str,
) -> None:
    part.visual(Sphere(radius=radius), origin=Origin(xyz=pos), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="accordion_drying_rack")

    metal = model.material("white_powder_coated_steel", color=(0.92, 0.94, 0.92, 1.0))
    hinge_metal = model.material("brushed_hinge_metal", color=(0.68, 0.70, 0.68, 1.0))
    rubber = model.material("dark_rubber_feet", color=(0.035, 0.035, 0.032, 1.0))

    center = model.part("center_frame")

    # Two A-shaped side frames, one at each end of the drying-bar span.
    for side, y in (("front", -HALF_WIDTH), ("rear", HALF_WIDTH)):
        apex = (0.0, y, CENTER_TOP_Z)
        foot_neg = (-0.24, y, CENTER_FOOT_Z)
        foot_pos = (0.24, y, CENTER_FOOT_Z)
        _tube(center, apex, foot_neg, radius=TUBE_RADIUS, material=metal, name=f"{side}_leg_0")
        _tube(center, apex, foot_pos, radius=TUBE_RADIUS, material=metal, name=f"{side}_leg_1")
        _tube(center, foot_neg, foot_pos, radius=TUBE_RADIUS, material=metal, name=f"{side}_foot_bar")
        _node(center, apex, radius=0.018, material=hinge_metal, name=f"{side}_apex_socket")
        _node(center, foot_neg, radius=0.021, material=rubber, name=f"{side}_foot_0")
        _node(center, foot_pos, radius=0.021, material=rubber, name=f"{side}_foot_1")

    # Main hinge ridge and central drying rods spanning the front/rear frames.
    _tube(
        center,
        (0.0, -HALF_WIDTH - 0.025, CENTER_TOP_Z),
        (0.0, HALF_WIDTH + 0.025, CENTER_TOP_Z),
        radius=HINGE_RADIUS,
        material=hinge_metal,
        name="top_hinge_ridge",
    )
    for i, t in enumerate((0.24, 0.42, 0.60, 0.78)):
        z = CENTER_TOP_Z * (1.0 - t) + CENTER_FOOT_Z * t
        x = 0.24 * t
        _tube(
            center,
            (x, -HALF_WIDTH, z),
            (x, HALF_WIDTH, z),
            radius=TUBE_RADIUS,
            material=metal,
            name=f"center_bar_pos_{i}",
        )
        _tube(
            center,
            (-x, -HALF_WIDTH, z),
            (-x, HALF_WIDTH, z),
            radius=TUBE_RADIUS,
            material=metal,
            name=f"center_bar_neg_{i}",
        )

    # Small hinge ears make the side-section pivots read as carried by the ridge.
    for sign, label in ((1.0, "pos"), (-1.0, "neg")):
        for y in (-HALF_WIDTH - 0.025, HALF_WIDTH + 0.025):
            _tube(
                center,
                (0.0, y, CENTER_TOP_Z),
                (sign * 0.030, y, CENTER_TOP_Z),
                radius=0.008,
                material=hinge_metal,
                name=f"{label}_hinge_ear_{'front' if y < 0 else 'rear'}",
            )

    # Lower hinge pins are mounted through the A-frame close to the feet; the
    # folding spreader links sit beside these pins and rotate about the same
    # front-to-rear hinge axes.
    for sign, label in ((1.0, "pos"), (-1.0, "neg")):
        _tube(
            center,
            (sign * 0.205, -HALF_WIDTH - 0.025, 0.125),
            (sign * 0.205, HALF_WIDTH + 0.025, 0.125),
            radius=HINGE_RADIUS,
            material=hinge_metal,
            name=f"lower_hinge_{label}",
        )

    def add_side_section(name: str, sign: float):
        section = model.part(name)
        local_y0 = -HALF_WIDTH + 0.025
        local_y1 = HALF_WIDTH - 0.025
        hinge_x = sign * (HINGE_RADIUS + 0.012)
        outer_x = sign * 0.92
        outer_z = -1.035

        _tube(
            section,
            (hinge_x, local_y0, 0.0),
            (hinge_x, local_y1, 0.0),
            radius=0.012,
            material=hinge_metal,
            name="hinge_sleeve",
        )
        _tube(
            section,
            (hinge_x, local_y0, 0.0),
            (outer_x, local_y0, outer_z),
            radius=TUBE_RADIUS,
            material=metal,
            name="side_rail_0",
        )
        _tube(
            section,
            (hinge_x, local_y1, 0.0),
            (outer_x, local_y1, outer_z),
            radius=TUBE_RADIUS,
            material=metal,
            name="side_rail_1",
        )
        _tube(
            section,
            (outer_x, local_y0, outer_z),
            (outer_x, local_y1, outer_z),
            radius=TUBE_RADIUS,
            material=metal,
            name="outer_crossbar",
        )
        for i, t in enumerate((0.14, 0.28, 0.42, 0.56, 0.70, 0.84)):
            x = hinge_x * (1.0 - t) + outer_x * t
            z = outer_z * t
            _tube(
                section,
                (x, local_y0, z),
                (x, local_y1, z),
                radius=0.0095,
                material=metal,
                name=f"drying_bar_{i}",
            )
        _node(section, (outer_x, local_y0, outer_z), radius=0.024, material=rubber, name="foot_0")
        _node(section, (outer_x, local_y1, outer_z), radius=0.024, material=rubber, name="foot_1")
        return section

    side_0 = add_side_section("side_section_0", 1.0)
    side_1 = add_side_section("side_section_1", -1.0)

    side_joint_0 = model.articulation(
        "center_to_side_0",
        ArticulationType.REVOLUTE,
        parent=center,
        child=side_0,
        origin=Origin(xyz=(0.0, 0.0, CENTER_TOP_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.05),
    )
    model.articulation(
        "center_to_side_1",
        ArticulationType.REVOLUTE,
        parent=center,
        child=side_1,
        origin=Origin(xyz=(0.0, 0.0, CENTER_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.05),
        mimic=Mimic(joint=side_joint_0.name, multiplier=1.0),
    )

    def add_spreader(name: str, sign: float):
        spreader = model.part(name)
        y0 = -HALF_WIDTH + 0.055
        y1 = HALF_WIDTH - 0.055
        sleeve_x = sign * (HINGE_RADIUS + 0.012)
        rail_start_x = sleeve_x
        outer_x = sign * 0.67
        _tube(
            spreader,
            (sleeve_x, y0, 0.0),
            (sleeve_x, y1, 0.0),
            radius=0.012,
            material=hinge_metal,
            name="hinge_sleeve",
        )
        _tube(
            spreader,
            (rail_start_x, y0, 0.0),
            (outer_x, y0, 0.0),
            radius=0.0095,
            material=metal,
            name="link_rail_0",
        )
        _tube(
            spreader,
            (rail_start_x, y1, 0.0),
            (outer_x, y1, 0.0),
            radius=0.0095,
            material=metal,
            name="link_rail_1",
        )
        _node(spreader, (sleeve_x, y0, 0.0), radius=0.014, material=hinge_metal, name="hinge_knuckle_0")
        _node(spreader, (sleeve_x, y1, 0.0), radius=0.014, material=hinge_metal, name="hinge_knuckle_1")
        _tube(
            spreader,
            (outer_x, y0, 0.0),
            (outer_x, y1, 0.0),
            radius=0.010,
            material=hinge_metal,
            name="outer_pivot_bar",
        )
        _node(spreader, (outer_x, y0, 0.0), radius=0.014, material=hinge_metal, name="outer_pin_0")
        _node(spreader, (outer_x, y1, 0.0), radius=0.014, material=hinge_metal, name="outer_pin_1")
        return spreader

    spreader_0 = add_spreader("lower_spreader_0", 1.0)
    spreader_1 = add_spreader("lower_spreader_1", -1.0)

    spreader_joint_0 = model.articulation(
        "center_to_spreader_0",
        ArticulationType.REVOLUTE,
        parent=center,
        child=spreader_0,
        origin=Origin(xyz=(0.205, 0.0, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.25),
    )
    model.articulation(
        "center_to_spreader_1",
        ArticulationType.REVOLUTE,
        parent=center,
        child=spreader_1,
        origin=Origin(xyz=(-0.205, 0.0, 0.125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.25),
        mimic=Mimic(joint=spreader_joint_0.name, multiplier=1.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    center = object_model.get_part("center_frame")
    side_0 = object_model.get_part("side_section_0")
    side_1 = object_model.get_part("side_section_1")
    spreader_0 = object_model.get_part("lower_spreader_0")
    spreader_1 = object_model.get_part("lower_spreader_1")
    side_joint = object_model.get_articulation("center_to_side_0")
    spreader_joint = object_model.get_articulation("center_to_spreader_0")

    ctx.expect_overlap(side_0, side_1, axes="y", min_overlap=0.62, name="side sections share drying-bar span")
    ctx.expect_overlap(
        side_0,
        center,
        axes="y",
        min_overlap=0.62,
        name="side hinge span aligns with center ridge",
    )
    ctx.expect_overlap(
        spreader_0,
        spreader_1,
        axes="y",
        min_overlap=0.60,
        name="lower spreaders stay matched across width",
    )

    side_origin = ctx.part_world_position(side_0)
    spreader_origin = ctx.part_world_position(spreader_0)
    ctx.check(
        "side section hinge is at top of A-frame",
        side_origin is not None and abs(side_origin[2] - CENTER_TOP_Z) < 0.01,
        details=f"side hinge origin={side_origin}",
    )
    ctx.check(
        "spreader hinge is near the feet",
        spreader_origin is not None and 0.08 <= spreader_origin[2] <= 0.18,
        details=f"spreader hinge origin={spreader_origin}",
    )

    rest_side = ctx.part_element_world_aabb(side_0, elem="outer_crossbar")
    rest_spreader = ctx.part_element_world_aabb(spreader_0, elem="outer_pivot_bar")
    with ctx.pose({side_joint: 0.85, spreader_joint: 0.85}):
        folded_side = ctx.part_element_world_aabb(side_0, elem="outer_crossbar")
        folded_mate = ctx.part_element_world_aabb(side_1, elem="outer_crossbar")
        folded_spreader = ctx.part_element_world_aabb(spreader_0, elem="outer_pivot_bar")
        folded_spreader_mate = ctx.part_element_world_aabb(spreader_1, elem="outer_pivot_bar")

    ctx.check(
        "side sections rotate upward on top hinges",
        rest_side is not None
        and folded_side is not None
        and folded_side[0][2] > rest_side[0][2] + 0.45,
        details=f"rest={rest_side}, folded={folded_side}",
    )
    ctx.check(
        "mimic keeps side sections aligned",
        folded_side is not None
        and folded_mate is not None
        and abs(folded_side[0][2] - folded_mate[0][2]) < 0.02,
        details=f"side_0={folded_side}, side_1={folded_mate}",
    )
    ctx.check(
        "lower spreaders rotate upward on lower hinges",
        rest_spreader is not None
        and folded_spreader is not None
        and folded_spreader[0][2] > rest_spreader[0][2] + 0.25,
        details=f"rest={rest_spreader}, folded={folded_spreader}",
    )
    ctx.check(
        "mimic keeps lower spreaders matched",
        folded_spreader is not None
        and folded_spreader_mate is not None
        and abs(folded_spreader[0][2] - folded_spreader_mate[0][2]) < 0.02,
        details=f"spreader_0={folded_spreader}, spreader_1={folded_spreader_mate}",
    )

    return ctx.report()


object_model = build_object_model()
