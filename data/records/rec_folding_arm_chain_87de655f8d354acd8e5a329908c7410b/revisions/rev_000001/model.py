from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


STRAP_WIDTH = 0.014
STRAP_THICKNESS = 0.012
INNER_LANE = 0.031
OUTER_LANE = 0.045
HINGE_HOLE_RADIUS = 0.0055


def _circle_profile(radius: float, *, center: tuple[float, float], segments: int = 28):
    cx, cy = center
    return [
        (cx + radius * cos(2.0 * pi * i / segments), cy + radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _shift_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _capsule_plate_mesh(length: float, *, name: str):
    outer = _shift_profile(
        rounded_rect_profile(length + STRAP_WIDTH, STRAP_WIDTH, STRAP_WIDTH * 0.49, corner_segments=8),
        length * 0.5,
        0.0,
    )
    holes = [
        _circle_profile(HINGE_HOLE_RADIUS, center=(0.0, 0.0)),
        _circle_profile(HINGE_HOLE_RADIUS, center=(length, 0.0)),
    ]
    geom = ExtrudeWithHolesGeometry(outer, holes, STRAP_THICKNESS, center=True)
    return mesh_from_geometry(geom, name)


def _add_vertical_cylinder(part, *, radius, length, xyz, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_flat_link(
    part,
    *,
    length: float,
    lane: float,
    plate_mesh,
    metal,
    dark,
    bronze,
    cover,
    prefix: str,
):
    """Author one moving chain link as two pierced flat straps with hardware."""
    span = 2.0 * lane + STRAP_WIDTH
    for side, y in (("pos", lane), ("neg", -lane)):
        part.visual(
            plate_mesh,
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=metal,
            name=f"{prefix}_strap_{side}",
        )
        part.visual(
            Box((length - 0.095, 0.0035, 0.006)),
            origin=Origin(xyz=(length * 0.5, y, STRAP_THICKNESS * 0.5 + 0.003)),
            material=dark,
            name=f"{prefix}_rib_{side}",
        )
        for x, end_name in ((0.0, "prox"), (length, "dist")):
            _add_vertical_cylinder(
                part,
                radius=0.0067,
                length=0.005,
                xyz=(x, y, STRAP_THICKNESS * 0.5 + 0.0025),
                material=bronze,
                name=f"{prefix}_{end_name}_bushing_{side}",
            )
            _add_vertical_cylinder(
                part,
                radius=0.0070,
                length=0.003,
                xyz=(x, y, STRAP_THICKNESS * 0.5 + 0.0065),
                material=dark,
                name=f"{prefix}_{end_name}_pin_head_{side}",
            )

    for x, tie_name in ((length * 0.34, "front_tie"), (length * 0.66, "rear_tie")):
        part.visual(
            Box((0.028, span, STRAP_THICKNESS)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=metal,
            name=f"{prefix}_{tie_name}",
        )

    # Removable inspection cover spanning the paired straps, with four cap screws.
    cover_x = length * 0.50
    part.visual(
        Box((0.145, span - 0.018, 0.0035)),
        origin=Origin(xyz=(cover_x, 0.0, STRAP_THICKNESS * 0.5 + 0.00175)),
        material=cover,
        name=f"{prefix}_access_cover",
    )
    for sx in (-0.052, 0.052):
        for sy in (-(span - 0.036) * 0.5, (span - 0.036) * 0.5):
            _add_vertical_cylinder(
                part,
                radius=0.0028,
                length=0.002,
                xyz=(cover_x + sx, sy, STRAP_THICKNESS * 0.5 + 0.0035),
                material=dark,
                name=f"{prefix}_cover_screw_{sx:+.2f}_{sy:+.2f}",
            )

    # Positive mechanical-stop blocks near each fold knuckle.
    for x, stop_name in ((0.055, "prox_stop"), (length - 0.055, "dist_stop")):
        part.visual(
            Box((0.030, 0.016, 0.018)),
            origin=Origin(xyz=(x, 0.0, STRAP_THICKNESS * 0.5 + 0.009)),
            material=dark,
            name=f"{prefix}_{stop_name}",
        )
        part.visual(
            Box((0.020, span - 0.016, 0.004)),
            origin=Origin(xyz=(x + (0.018 if x < length * 0.5 else -0.018), 0.0, STRAP_THICKNESS * 0.5 + 0.004)),
            material=bronze,
            name=f"{prefix}_{stop_name}_wear_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_chain_study")

    bead_blast = model.material("bead_blast_steel", rgba=(0.56, 0.58, 0.56, 1.0))
    dark_oxide = model.material("black_oxide_hardware", rgba=(0.045, 0.047, 0.045, 1.0))
    bronze = model.material("oilite_bronze_bushings", rgba=(0.72, 0.47, 0.20, 1.0))
    cover_mat = model.material("brushed_access_covers", rgba=(0.37, 0.39, 0.38, 1.0))
    raw_plate = model.material("raw_fixture_plate", rgba=(0.30, 0.32, 0.33, 1.0))

    plate_0 = _capsule_plate_mesh(0.520, name="link_0_pierced_plate")
    plate_1 = _capsule_plate_mesh(0.465, name="link_1_pierced_plate")
    plate_2 = _capsule_plate_mesh(0.480, name="link_2_pierced_plate")
    plate_3 = _capsule_plate_mesh(0.455, name="link_3_pierced_plate")

    base = model.part("base_frame")
    base.visual(
        Box((0.420, 0.250, 0.026)),
        origin=Origin(xyz=(0.075, 0.0, -0.060)),
        material=raw_plate,
        name="ground_plate",
    )
    base.visual(
        Box((0.180, 0.110, 0.018)),
        origin=Origin(xyz=(0.000, 0.0, -0.038)),
        material=bead_blast,
        name="raised_boss",
    )
    for y, side in ((0.057, "pos"), (-0.057, "neg")):
        base.visual(
            Box((0.090, 0.010, 0.086)),
            origin=Origin(xyz=(0.0, y, -0.008)),
            material=bead_blast,
            name=f"base_cheek_{side}",
        )
        _add_vertical_cylinder(
            base,
            radius=0.013,
            length=0.008,
            xyz=(0.0, y, 0.039),
            material=bronze,
            name=f"base_outer_bearing_{side}",
        )
        base.visual(
            Box((0.026, 0.022, 0.030)),
            origin=Origin(xyz=(0.044, y, 0.024)),
            material=dark_oxide,
            name=f"base_stop_block_{side}",
        )
    for x, y in ((-0.110, -0.080), (-0.110, 0.080), (0.240, -0.080), (0.240, 0.080)):
        _add_vertical_cylinder(
            base,
            radius=0.009,
            length=0.006,
            xyz=(x, y, -0.044),
            material=dark_oxide,
            name=f"base_anchor_{x:+.2f}_{y:+.2f}",
        )
    base.visual(
        Box((0.145, 0.080, 0.004)),
        origin=Origin(xyz=(0.110, 0.0, -0.027)),
        material=cover_mat,
        name="base_access_cover",
    )

    link_0 = model.part("link_0")
    _add_flat_link(
        link_0,
        length=0.520,
        lane=OUTER_LANE,
        plate_mesh=plate_0,
        metal=bead_blast,
        dark=dark_oxide,
        bronze=bronze,
        cover=cover_mat,
        prefix="link_0",
    )

    link_1 = model.part("link_1")
    _add_flat_link(
        link_1,
        length=0.465,
        lane=INNER_LANE,
        plate_mesh=plate_1,
        metal=bead_blast,
        dark=dark_oxide,
        bronze=bronze,
        cover=cover_mat,
        prefix="link_1",
    )

    link_2 = model.part("link_2")
    _add_flat_link(
        link_2,
        length=0.480,
        lane=OUTER_LANE,
        plate_mesh=plate_2,
        metal=bead_blast,
        dark=dark_oxide,
        bronze=bronze,
        cover=cover_mat,
        prefix="link_2",
    )

    link_3 = model.part("link_3")
    _add_flat_link(
        link_3,
        length=0.455,
        lane=INNER_LANE,
        plate_mesh=plate_3,
        metal=bead_blast,
        dark=dark_oxide,
        bronze=bronze,
        cover=cover_mat,
        prefix="link_3",
    )

    end_bracket = model.part("end_bracket")
    for y, side in ((OUTER_LANE, "pos"), (-OUTER_LANE, "neg")):
        end_bracket.visual(
            Box((0.235, STRAP_WIDTH, STRAP_THICKNESS)),
            origin=Origin(xyz=(0.118, y, 0.0)),
            material=bead_blast,
            name=f"end_fork_rail_{side}",
        )
        _add_vertical_cylinder(
            end_bracket,
            radius=0.011,
            length=0.005,
            xyz=(0.0, y, STRAP_THICKNESS * 0.5 + 0.0025),
            material=bronze,
            name=f"end_bearing_{side}",
        )
    end_bracket.visual(
        Box((0.190, 2.0 * OUTER_LANE + STRAP_WIDTH, 0.010)),
        origin=Origin(xyz=(0.130, 0.0, 0.001)),
        material=raw_plate,
        name="end_mounting_blade",
    )
    end_bracket.visual(
        Box((0.045, 2.0 * OUTER_LANE + STRAP_WIDTH, 0.024)),
        origin=Origin(xyz=(0.220, 0.0, 0.010)),
        material=dark_oxide,
        name="end_cross_stop",
    )
    end_bracket.visual(
        Box((0.125, 0.064, 0.004)),
        origin=Origin(xyz=(0.135, 0.0, STRAP_THICKNESS * 0.5 + 0.002)),
        material=cover_mat,
        name="end_access_cover",
    )
    for x in (0.095, 0.175):
        for y in (-0.024, 0.024):
            _add_vertical_cylinder(
                end_bracket,
                radius=0.0028,
                length=0.002,
                xyz=(x, y, STRAP_THICKNESS * 0.5 + 0.0045),
                material=dark_oxide,
                name=f"end_cover_screw_{x:.2f}_{y:+.2f}",
            )

    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=1.0, lower=-0.62, upper=0.62),
        meta={"mechanical_stops": "visible cheek blocks on the base frame limit rotation"},
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.520, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.68, upper=0.68),
        meta={"mechanical_stops": "opposed hard-stop lugs flank the knuckle"},
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.465, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.68, upper=0.68),
        meta={"mechanical_stops": "interleaved bearing pads bottom against stop blocks"},
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=-0.68, upper=0.68),
        meta={"mechanical_stops": "machined tabs provide a limited fold envelope"},
    )
    model.articulation(
        "link_3_to_end",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=end_bracket,
        origin=Origin(xyz=(0.455, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.0, lower=-0.58, upper=0.58),
        meta={"mechanical_stops": "terminal cross stop arrests the nested fork"},
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fold_joints = [
        object_model.get_articulation("base_to_link_0"),
        object_model.get_articulation("link_0_to_link_1"),
        object_model.get_articulation("link_1_to_link_2"),
        object_model.get_articulation("link_2_to_link_3"),
        object_model.get_articulation("link_3_to_end"),
    ]
    ctx.check(
        "all fold joints are explicit limited hinges",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower < 0.0 < joint.motion_limits.upper
            for joint in fold_joints
        ),
        details=str([(joint.name, joint.articulation_type, joint.motion_limits) for joint in fold_joints]),
    )

    base = object_model.get_part("base_frame")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    end_bracket = object_model.get_part("end_bracket")
    ctx.expect_overlap(
        base,
        link_0,
        axes="z",
        min_overlap=0.010,
        elem_a="base_cheek_pos",
        elem_b="link_0_strap_pos",
        name="base cheek captures first outer strap vertically",
    )
    ctx.expect_overlap(
        link_0,
        link_1,
        axes="z",
        min_overlap=0.010,
        elem_a="link_0_strap_pos",
        elem_b="link_1_strap_pos",
        name="nested link straps share the same bearing plane",
    )

    rest_tip = ctx.part_world_position(end_bracket)
    with ctx.pose(
        {
            fold_joints[0]: 0.35,
            fold_joints[1]: -0.30,
            fold_joints[2]: 0.32,
            fold_joints[3]: -0.28,
            fold_joints[4]: 0.24,
        }
    ):
        moved_tip = ctx.part_world_position(end_bracket)
    ctx.check(
        "coordinated fold pose moves terminal bracket",
        rest_tip is not None
        and moved_tip is not None
        and abs(moved_tip[0] - rest_tip[0]) + abs(moved_tip[1] - rest_tip[1]) > 0.050,
        details=f"rest={rest_tip}, moved={moved_tip}",
    )

    return ctx.report()


object_model = build_object_model()
