from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


AXIS_Z = 0.28


def _box(part, size, center, name: str, material: Material) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), name=name, material=material)


def _bolt_x(part, x: float, y: float, z: float, name: str, material: Material) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0)),
        name=name,
        material=material,
    )


def _add_rect_sleeve(
    part,
    *,
    span_x: tuple[float, float],
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    wall_material: Material,
    pad_material: Material,
    prefix: str,
    collar: bool = True,
) -> None:
    """Build a rigid open rectangular sleeve from connected wall members."""
    if prefix == "outer":
        names = {
            "top_wall": "outer_top_wall",
            "bottom_wall": "outer_bottom_wall",
            "side_wall_0": "outer_side_wall_0",
            "side_wall_1": "outer_side_wall_1",
            "inner_top_pad": "outer_inner_top_pad",
            "inner_bottom_pad": "outer_inner_bottom_pad",
            "inner_side_pad_0": "outer_inner_side_pad_0",
            "inner_side_pad_1": "outer_inner_side_pad_1",
        }
    elif prefix == "middle":
        names = {
            "top_wall": "middle_top_wall",
            "bottom_wall": "middle_bottom_wall",
            "side_wall_0": "middle_side_wall_0",
            "side_wall_1": "middle_side_wall_1",
            "inner_top_pad": "middle_inner_top_pad",
            "inner_bottom_pad": "middle_inner_bottom_pad",
            "inner_side_pad_0": "middle_inner_side_pad_0",
            "inner_side_pad_1": "middle_inner_side_pad_1",
        }
    elif prefix == "inner":
        names = {
            "top_wall": "inner_top_wall",
            "bottom_wall": "inner_bottom_wall",
            "side_wall_0": "inner_side_wall_0",
            "side_wall_1": "inner_side_wall_1",
            "inner_top_pad": "inner_inner_top_pad",
            "inner_bottom_pad": "inner_inner_bottom_pad",
            "inner_side_pad_0": "inner_inner_side_pad_0",
            "inner_side_pad_1": "inner_inner_side_pad_1",
        }
    else:
        names = {
            "top_wall": f"{prefix}_top_wall",
            "bottom_wall": f"{prefix}_bottom_wall",
            "side_wall_0": f"{prefix}_side_wall_0",
            "side_wall_1": f"{prefix}_side_wall_1",
            "inner_top_pad": f"{prefix}_inner_top_pad",
            "inner_bottom_pad": f"{prefix}_inner_bottom_pad",
            "inner_side_pad_0": f"{prefix}_inner_side_pad_0",
            "inner_side_pad_1": f"{prefix}_inner_side_pad_1",
        }
    x0, x1 = span_x
    length = x1 - x0
    xc = (x0 + x1) / 2.0
    side_t = (outer_y - inner_y) / 2.0
    cap_t = (outer_z - inner_z) / 2.0

    _box(
        part,
        (length, outer_y, cap_t),
        (xc, 0.0, inner_z / 2.0 + cap_t / 2.0),
        names["top_wall"],
        wall_material,
    )
    _box(
        part,
        (length, outer_y, cap_t),
        (xc, 0.0, -inner_z / 2.0 - cap_t / 2.0),
        names["bottom_wall"],
        wall_material,
    )
    _box(
        part,
        (length, side_t, inner_z),
        (xc, inner_y / 2.0 + side_t / 2.0, 0.0),
        names["side_wall_0"],
        wall_material,
    )
    _box(
        part,
        (length, side_t, inner_z),
        (xc, -inner_y / 2.0 - side_t / 2.0, 0.0),
        names["side_wall_1"],
        wall_material,
    )

    # Bronze guide pads are mounted to the inside of the front bushing but leave
    # real clearance for the next sliding stage.
    pad_x = x1 - 0.040
    pad_l = 0.060
    pad_t = 0.003
    _box(
        part,
        (pad_l, 0.052, pad_t),
        (pad_x, 0.0, inner_z / 2.0 - pad_t / 2.0),
        names["inner_top_pad"],
        pad_material,
    )
    _box(
        part,
        (pad_l, 0.052, pad_t),
        (pad_x, 0.0, -inner_z / 2.0 + pad_t / 2.0),
        names["inner_bottom_pad"],
        pad_material,
    )
    _box(
        part,
        (pad_l, pad_t, 0.038),
        (pad_x, inner_y / 2.0 - pad_t / 2.0, 0.0),
        names["inner_side_pad_0"],
        pad_material,
    )
    _box(
        part,
        (pad_l, pad_t, 0.038),
        (pad_x, -inner_y / 2.0 + pad_t / 2.0, 0.0),
        names["inner_side_pad_1"],
        pad_material,
    )

    # Outer bearing strips ride inside the parent sleeve/housing.
    strip_l = 0.080
    for i, strip_x in enumerate((x0 + 0.070, x1 - 0.075)):
        _box(
            part,
            (strip_l, 0.050, 0.004),
            (strip_x, 0.0, outer_z / 2.0 + 0.002),
            f"{prefix}_outer_top_pad_{i}",
            pad_material,
        )
        _box(
            part,
            (strip_l, 0.050, 0.004),
            (strip_x, 0.0, -outer_z / 2.0 - 0.002),
            f"{prefix}_outer_bottom_pad_{i}",
            pad_material,
        )
        _box(
            part,
            (strip_l, 0.004, 0.044),
            (strip_x, outer_y / 2.0 + 0.002, 0.0),
            f"{prefix}_outer_side_pad_{i}_0",
            pad_material,
        )
        _box(
            part,
            (strip_l, 0.004, 0.044),
            (strip_x, -outer_y / 2.0 - 0.002, 0.0),
            f"{prefix}_outer_side_pad_{i}_1",
            pad_material,
        )

    if collar:
        # A welded front guide bushing/flange. It is still open in the middle so
        # the next sleeve moves through a rigid rectangular guide, not a hose.
        c_l = 0.032
        c_x = x1 - c_l / 2.0
        c_outer_y = outer_y + 0.014
        c_outer_z = outer_z + 0.014
        _box(
            part,
            (c_l, c_outer_y, cap_t + 0.012),
            (c_x, 0.0, inner_z / 2.0 + (cap_t + 0.012) / 2.0),
            f"{prefix}_front_bushing_top",
            wall_material,
        )
        _box(
            part,
            (c_l, c_outer_y, cap_t + 0.012),
            (c_x, 0.0, -inner_z / 2.0 - (cap_t + 0.012) / 2.0),
            f"{prefix}_front_bushing_bottom",
            wall_material,
        )
        _box(
            part,
            (c_l, side_t + 0.010, inner_z),
            (c_x, inner_y / 2.0 + (side_t + 0.010) / 2.0, 0.0),
            f"{prefix}_front_bushing_side_0",
            wall_material,
        )
        _box(
            part,
            (c_l, side_t + 0.010, inner_z),
            (c_x, -inner_y / 2.0 - (side_t + 0.010) / 2.0, 0.0),
            f"{prefix}_front_bushing_side_1",
            wall_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_ram_stack")

    painted = model.material("painted_housing", color=(0.12, 0.15, 0.17, 1.0))
    dark_steel = model.material("dark_sliding_steel", color=(0.22, 0.24, 0.25, 1.0))
    mid_steel = model.material("brushed_inner_steel", color=(0.42, 0.44, 0.43, 1.0))
    bright_steel = model.material("polished_tip_steel", color=(0.66, 0.68, 0.66, 1.0))
    bronze = model.material("bronze_wear_pads", color=(0.78, 0.55, 0.25, 1.0))
    black = model.material("black_seals", color=(0.02, 0.025, 0.025, 1.0))
    bolt_mat = model.material("dark_fasteners", color=(0.04, 0.045, 0.045, 1.0))

    base = model.part("base_housing")

    # Base housing: a compact rigid guide box, open at the ram end and carried
    # by a bolted plinth. The sliding axis is along +X through AXIS_Z.
    base_x0, base_x1 = 0.0, 0.39
    base_l = base_x1 - base_x0
    outer_y = 0.360
    inner_y = 0.270
    inner_z = 0.205
    cap_t = 0.036
    outer_z = inner_z + 2.0 * cap_t
    xc = (base_x0 + base_x1) / 2.0
    _box(base, (0.48, 0.44, 0.140), (0.20, 0.0, 0.070), "floor_plinth", painted)
    _box(
        base,
        (base_l, outer_y, cap_t),
        (xc, 0.0, AXIS_Z + inner_z / 2.0 + cap_t / 2.0),
        "housing_top_wall",
        painted,
    )
    _box(
        base,
        (base_l, outer_y, cap_t),
        (xc, 0.0, AXIS_Z - inner_z / 2.0 - cap_t / 2.0),
        "housing_bottom_wall",
        painted,
    )
    side_t = (outer_y - inner_y) / 2.0
    _box(
        base,
        (base_l, side_t, inner_z),
        (xc, inner_y / 2.0 + side_t / 2.0, AXIS_Z),
        "housing_side_wall_0",
        painted,
    )
    _box(
        base,
        (base_l, side_t, inner_z),
        (xc, -inner_y / 2.0 - side_t / 2.0, AXIS_Z),
        "housing_side_wall_1",
        painted,
    )
    _box(base, (0.030, outer_y, outer_z), (0.015, 0.0, AXIS_Z), "rear_cap", painted)

    # Front bolted flange ring around the open guide throat.
    flange_l = 0.032
    flange_x = base_x1 - flange_l / 2.0
    flange_outer_y = 0.392
    flange_outer_z = 0.302
    _box(
        base,
        (flange_l, flange_outer_y, 0.040),
        (flange_x, 0.0, AXIS_Z + inner_z / 2.0 + 0.020),
        "front_flange_top",
        painted,
    )
    _box(
        base,
        (flange_l, flange_outer_y, 0.040),
        (flange_x, 0.0, AXIS_Z - inner_z / 2.0 - 0.020),
        "front_flange_bottom",
        painted,
    )
    _box(
        base,
        (flange_l, 0.050, inner_z),
        (flange_x, inner_y / 2.0 + 0.025, AXIS_Z),
        "front_flange_side_0",
        painted,
    )
    _box(
        base,
        (flange_l, 0.050, inner_z),
        (flange_x, -inner_y / 2.0 - 0.025, AXIS_Z),
        "front_flange_side_1",
        painted,
    )
    _box(base, (0.024, 0.250, 0.012), (base_x1 + 0.006, 0.0, AXIS_Z + 0.104), "upper_wiper", black)
    _box(base, (0.024, 0.250, 0.012), (base_x1 + 0.006, 0.0, AXIS_Z - 0.104), "lower_wiper", black)
    _box(base, (0.024, 0.012, 0.185), (base_x1 + 0.006, 0.136, AXIS_Z), "side_wiper_0", black)
    _box(base, (0.024, 0.012, 0.185), (base_x1 + 0.006, -0.136, AXIS_Z), "side_wiper_1", black)

    # Internal bronze pads are just inside the base guide and clear the largest
    # sleeve by several millimetres over the whole prismatic path.
    for px, top_name, bottom_name, side_0_name, side_1_name in (
        (0.100, "base_top_pad_0", "base_bottom_pad_0", "base_side_pad_0_0", "base_side_pad_0_1"),
        (0.315, "base_top_pad_1", "base_bottom_pad_1", "base_side_pad_1_0", "base_side_pad_1_1"),
    ):
        _box(base, (0.070, 0.070, 0.005), (px, 0.0, AXIS_Z + 0.100), top_name, bronze)
        _box(base, (0.070, 0.070, 0.005), (px, 0.0, AXIS_Z - 0.100), bottom_name, bronze)
        _box(base, (0.070, 0.005, 0.060), (px, 0.133, AXIS_Z), side_0_name, bronze)
        _box(base, (0.070, 0.005, 0.060), (px, -0.133, AXIS_Z), side_1_name, bronze)

    for i, (by, bz) in enumerate(((0.155, 0.405), (-0.155, 0.405), (0.155, 0.155), (-0.155, 0.155))):
        _bolt_x(base, base_x1 + 0.004, by, bz, f"flange_bolt_{i}", bolt_mat)
    for i, by in enumerate((-0.160, 0.160)):
        _box(base, (0.070, 0.050, 0.012), (0.06, by, 0.146), f"plinth_rib_{i}", painted)
        _box(base, (0.070, 0.050, 0.012), (0.34, by, 0.146), f"plinth_rib_{i+2}", painted)

    sleeve_0 = model.part("sleeve_0")
    _add_rect_sleeve(
        sleeve_0,
        span_x=(-0.250, 0.210),
        outer_y=0.232,
        outer_z=0.168,
        inner_y=0.184,
        inner_z=0.130,
        wall_material=dark_steel,
        pad_material=bronze,
        prefix="outer",
    )
    _box(sleeve_0, (0.018, 0.250, 0.014), (-0.246, 0.0, 0.091), "rear_stop_top", black)
    _box(sleeve_0, (0.018, 0.250, 0.014), (-0.246, 0.0, -0.091), "rear_stop_bottom", black)

    sleeve_1 = model.part("sleeve_1")
    _add_rect_sleeve(
        sleeve_1,
        span_x=(-0.200, 0.190),
        outer_y=0.166,
        outer_z=0.114,
        inner_y=0.130,
        inner_z=0.086,
        wall_material=mid_steel,
        pad_material=bronze,
        prefix="middle",
    )
    _box(sleeve_1, (0.016, 0.184, 0.012), (-0.197, 0.0, 0.063), "rear_stop_top", black)
    _box(sleeve_1, (0.016, 0.184, 0.012), (-0.197, 0.0, -0.063), "rear_stop_bottom", black)

    sleeve_2 = model.part("sleeve_2")
    _add_rect_sleeve(
        sleeve_2,
        span_x=(-0.170, 0.170),
        outer_y=0.112,
        outer_z=0.074,
        inner_y=0.088,
        inner_z=0.052,
        wall_material=bright_steel,
        pad_material=bronze,
        prefix="inner",
        collar=False,
    )
    _box(sleeve_2, (0.014, 0.128, 0.006), (-0.168, 0.0, 0.0400), "rear_stop_top", black)
    _box(sleeve_2, (0.014, 0.128, 0.006), (-0.168, 0.0, -0.0400), "rear_stop_bottom", black)
    _box(sleeve_2, (0.028, 0.154, 0.106), (0.184, 0.0, 0.0), "tip_plate", bright_steel)
    _box(sleeve_2, (0.008, 0.118, 0.070), (0.166, 0.0, 0.0), "tip_weld_pad", dark_steel)
    for i, (by, bz) in enumerate(((0.058, 0.036), (-0.058, 0.036), (0.058, -0.036), (-0.058, -0.036))):
        _bolt_x(sleeve_2, 0.202, by, bz, f"tip_bolt_{i}", bolt_mat)

    model.articulation(
        "base_to_sleeve_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=sleeve_0,
        origin=Origin(xyz=(base_x1, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.20, lower=0.0, upper=0.150),
    )
    model.articulation(
        "sleeve_0_to_sleeve_1",
        ArticulationType.PRISMATIC,
        parent=sleeve_0,
        child=sleeve_1,
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=700.0, velocity=0.18, lower=0.0, upper=0.130),
    )
    model.articulation(
        "sleeve_1_to_sleeve_2",
        ArticulationType.PRISMATIC,
        parent=sleeve_1,
        child=sleeve_2,
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=520.0, velocity=0.16, lower=0.0, upper=0.110),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve_0 = object_model.get_part("sleeve_0")
    sleeve_1 = object_model.get_part("sleeve_1")
    sleeve_2 = object_model.get_part("sleeve_2")
    base = object_model.get_part("base_housing")
    j0 = object_model.get_articulation("base_to_sleeve_0")
    j1 = object_model.get_articulation("sleeve_0_to_sleeve_1")
    j2 = object_model.get_articulation("sleeve_1_to_sleeve_2")

    ctx.check("three prismatic stages", len(object_model.articulations) == 3, "ram should expose exactly three sliding stages")
    for joint in (j0, j1, j2):
        ctx.check(
            f"{joint.name} slides on common x axis",
            joint.articulation_type == ArticulationType.PRISMATIC and tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    # Each nested member keeps meaningful insertion length at rest.
    ctx.expect_overlap(sleeve_0, base, axes="x", min_overlap=0.240, name="outer sleeve is deeply guided in housing")
    ctx.expect_overlap(sleeve_1, sleeve_0, axes="x", min_overlap=0.220, name="middle sleeve is deeply guided in outer sleeve")
    ctx.expect_overlap(sleeve_2, sleeve_1, axes="x", min_overlap=0.210, name="inner sleeve is deeply guided in middle sleeve")
    ctx.expect_within(sleeve_0, base, axes="yz", margin=0.0, name="outer sleeve fits within housing guide envelope")
    ctx.expect_within(sleeve_1, sleeve_0, axes="yz", margin=0.0, name="middle sleeve fits within outer sleeve envelope")
    ctx.expect_within(sleeve_2, sleeve_1, axes="yz", margin=0.0, name="inner sleeve fits within middle sleeve envelope")
    ctx.expect_gap(
        base,
        sleeve_0,
        axis="z",
        positive_elem="base_top_pad_0",
        negative_elem="outer_top_wall",
        min_gap=0.006,
        name="outer sleeve clears upper housing guide",
    )
    ctx.expect_gap(
        sleeve_0,
        base,
        axis="z",
        positive_elem="outer_bottom_wall",
        negative_elem="base_bottom_pad_0",
        min_gap=0.006,
        name="outer sleeve clears lower housing guide",
    )
    ctx.expect_gap(
        base,
        sleeve_0,
        axis="y",
        positive_elem="base_side_pad_0_0",
        negative_elem="outer_side_wall_0",
        min_gap=0.006,
        name="outer sleeve clears housing side guide",
    )
    ctx.expect_gap(
        sleeve_0,
        sleeve_1,
        axis="z",
        positive_elem="outer_inner_top_pad",
        negative_elem="middle_top_wall",
        min_gap=0.002,
        name="middle sleeve clears outer upper bushing",
    )
    ctx.expect_gap(
        sleeve_1,
        sleeve_0,
        axis="z",
        positive_elem="middle_bottom_wall",
        negative_elem="outer_inner_bottom_pad",
        min_gap=0.002,
        name="middle sleeve clears outer lower bushing",
    )
    ctx.expect_gap(
        sleeve_1,
        sleeve_2,
        axis="z",
        positive_elem="middle_inner_top_pad",
        negative_elem="inner_top_wall",
        min_gap=0.001,
        name="inner sleeve clears middle upper bushing",
    )
    ctx.expect_gap(
        sleeve_2,
        sleeve_1,
        axis="z",
        positive_elem="inner_bottom_wall",
        negative_elem="middle_inner_bottom_pad",
        min_gap=0.001,
        name="inner sleeve clears middle lower bushing",
    )

    rest_tip = ctx.part_world_aabb(sleeve_2)
    rest_x = rest_tip[1][0] if rest_tip else None
    with ctx.pose({j0: 0.150, j1: 0.130, j2: 0.110}):
        ctx.expect_overlap(sleeve_0, base, axes="x", min_overlap=0.095, name="outer sleeve retains guide length at full travel")
        ctx.expect_overlap(sleeve_1, sleeve_0, axes="x", min_overlap=0.120, name="middle sleeve retains guide length at full travel")
        ctx.expect_overlap(sleeve_2, sleeve_1, axes="x", min_overlap=0.110, name="inner sleeve retains guide length at full travel")
        ctx.expect_within(sleeve_0, base, axes="yz", margin=0.0, name="outer sleeve stays centered at full travel")
        ctx.expect_within(sleeve_1, sleeve_0, axes="yz", margin=0.0, name="middle sleeve stays centered at full travel")
        ctx.expect_within(sleeve_2, sleeve_1, axes="yz", margin=0.0, name="inner sleeve stays centered at full travel")
        full_tip = ctx.part_world_aabb(sleeve_2)
        full_x = full_tip[1][0] if full_tip else None

    ctx.check(
        "stack extends outward",
        rest_x is not None and full_x is not None and full_x > rest_x + 0.35,
        details=f"rest_tip_x={rest_x}, full_tip_x={full_x}",
    )

    return ctx.report()


object_model = build_object_model()
