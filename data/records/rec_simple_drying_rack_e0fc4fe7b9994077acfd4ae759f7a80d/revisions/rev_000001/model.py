from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _closed_tube_loop(name: str, points, *, radius: float, corner_radius: float):
    return _mesh(
        name,
        wire_from_points(
            points,
            radius=radius,
            radial_segments=18,
            closed_path=True,
            corner_mode="fillet",
            corner_radius=corner_radius,
            corner_segments=10,
        ),
    )


def _open_tube(name: str, points, *, radius: float, corner_radius: float):
    return _mesh(
        name,
        wire_from_points(
            points,
            radius=radius,
            radial_segments=18,
            closed_path=False,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=corner_radius,
            corner_segments=10,
        ),
    )


def _add_bolts_on_x_face(
    part,
    *,
    x: float,
    ys: tuple[float, ...],
    zs: tuple[float, ...],
    material,
    prefix: str,
    length: float = 0.006,
):
    for row_index, y in enumerate(ys):
        for col_index, z in enumerate(zs):
            part.visual(
                Cylinder(radius=0.004, length=length),
                origin=Origin(
                    xyz=(x, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=material,
                name=f"{prefix}_{row_index}_{col_index}",
            )


def _add_bolts_on_z_face(
    part,
    *,
    z: float,
    xs: tuple[float, ...],
    ys: tuple[float, ...],
    material,
    prefix: str,
):
    for row_index, x in enumerate(xs):
        for col_index, y in enumerate(ys):
            part.visual(
                Cylinder(radius=0.0038, length=0.008),
                origin=Origin(xyz=(x, y, z)),
                material=material,
                name=f"{prefix}_{row_index}_{col_index}",
            )


def _add_wing(
    part,
    *,
    mesh_name: str,
    sign: float,
    tube_radius: float,
    rack_depth: float,
    wing_reach: float,
    frame_material,
    adapter_material,
    fastener_material,
):
    inner_x = sign * 0.045
    outer_x = sign * wing_reach
    frame_points = [
        (inner_x, -rack_depth / 2.0, 0.0),
        (outer_x, -rack_depth / 2.0, 0.0),
        (outer_x, rack_depth / 2.0, 0.0),
        (inner_x, rack_depth / 2.0, 0.0),
    ]
    part.visual(
        _closed_tube_loop(
            f"{mesh_name}_loop",
            frame_points,
            radius=tube_radius,
            corner_radius=0.030,
        ),
        material=frame_material,
        name="wing_loop",
    )

    rail_positions = (
        sign * 0.090,
        sign * 0.145,
        sign * 0.200,
        sign * 0.255,
    )
    for rail_index, x_pos in enumerate(rail_positions):
        part.visual(
            Cylinder(radius=0.0045, length=rack_depth + 0.02),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=frame_material,
            name="outer_hanging_rail" if rail_index == len(rail_positions) - 1 else f"hanging_rail_{rail_index}",
        )

    part.visual(
        Cylinder(radius=0.009, length=0.24),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=adapter_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.046, 0.13, 0.026)),
        origin=Origin(xyz=(sign * 0.023, 0.0, 0.0)),
        material=adapter_material,
        name="adapter_bridge",
    )
    part.visual(
        Box((0.022, 0.12, 0.014)),
        origin=Origin(xyz=(sign * 0.018, 0.0, -0.017)),
        material=adapter_material,
        name="stop_tab",
    )
    for brace_index, y_pos in enumerate((-0.135, 0.135)):
        part.visual(
            Box((0.060, 0.030, 0.012)),
            origin=Origin(xyz=(sign * 0.058, y_pos, 0.0)),
            material=adapter_material,
            name=f"brace_plate_{brace_index}",
        )

    _add_bolts_on_z_face(
        part,
        z=0.017,
        xs=(sign * 0.014, sign * 0.026),
        ys=(-0.055, 0.055),
        material=fastener_material,
        prefix=f"{mesh_name}_adapter_bolt",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_drying_rack")

    galvanized = model.material("galvanized", rgba=(0.70, 0.72, 0.74, 1.0))
    enamel = model.material("enamel", rgba=(0.57, 0.60, 0.54, 1.0))
    fastener = model.material("fastener", rgba=(0.47, 0.48, 0.50, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    rack_depth = 0.68
    center_width = 0.46
    wing_reach = 0.30
    top_z = 0.84
    tube_radius = 0.011

    center_frame = model.part("center_frame")
    center_frame.inertial = Inertial.from_geometry(
        Box((0.62, 0.72, 0.86)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
    )
    center_frame.visual(
        _closed_tube_loop(
            "center_top_frame",
            [
                (-center_width / 2.0, -rack_depth / 2.0, top_z),
                (-center_width / 2.0, rack_depth / 2.0, top_z),
                (center_width / 2.0, rack_depth / 2.0, top_z),
                (center_width / 2.0, -rack_depth / 2.0, top_z),
            ],
            radius=tube_radius,
            corner_radius=0.045,
        ),
        material=galvanized,
        name="main_top_frame",
    )

    center_rail_positions = (-0.15, -0.075, 0.0, 0.075, 0.15)
    for rail_index, x_pos in enumerate(center_rail_positions):
        center_frame.visual(
            Cylinder(radius=0.0048, length=rack_depth + 0.02),
            origin=Origin(xyz=(x_pos, 0.0, top_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name="center_mid_rail" if rail_index == 2 else f"center_rail_{rail_index}",
        )

    for side_name, x_pos in (("left", -0.11), ("right", 0.11)):
        center_frame.visual(
            _open_tube(
                f"{side_name}_support_loop",
                [
                    (x_pos, -0.26, 0.79),
                    (x_pos, -0.29, 0.03),
                    (x_pos, 0.29, 0.03),
                    (x_pos, 0.26, 0.79),
                ],
                radius=0.012,
                corner_radius=0.060,
            ),
            material=galvanized,
            name=f"{side_name}_support_loop",
        )
        top_edge_x = -center_width / 2.0 if x_pos < 0.0 else center_width / 2.0
        for brace_name, y_pos in (("front", -0.26), ("rear", 0.26)):
            center_frame.visual(
                _open_tube(
                    f"{side_name}_{brace_name}_shoulder_brace",
                    [
                        (x_pos, y_pos, 0.79),
                        (top_edge_x, y_pos, top_z),
                    ],
                    radius=0.009,
                    corner_radius=0.0,
                ),
                material=enamel,
                name=f"{side_name}_{brace_name}_shoulder_brace",
            )

    center_frame.visual(
        Cylinder(radius=0.009, length=0.24),
        origin=Origin(xyz=(0.0, -0.18, 0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel,
        name="lower_cross_brace",
    )
    center_frame.visual(
        Cylinder(radius=0.008, length=0.23),
        origin=Origin(xyz=(0.0, 0.18, 0.03), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=enamel,
        name="mid_cross_brace",
    )

    for x_pos in (-0.11, 0.11):
        for y_pos in (-0.29, 0.29):
            center_frame.visual(
                Box((0.046, 0.082, 0.028)),
                origin=Origin(xyz=(x_pos, y_pos, 0.024)),
                material=enamel,
                name=f"foot_socket_{'left' if x_pos < 0.0 else 'right'}_{'front' if y_pos < 0.0 else 'rear'}",
            )
            center_frame.visual(
                Cylinder(radius=0.014, length=0.026),
                origin=Origin(xyz=(x_pos, y_pos, 0.022)),
                material=enamel,
                name=f"foot_sleeve_{'left' if x_pos < 0.0 else 'right'}_{'front' if y_pos < 0.0 else 'rear'}",
            )
            center_frame.visual(
                Box((0.038, 0.075, 0.019)),
                origin=Origin(xyz=(x_pos, y_pos, 0.0095)),
                material=rubber,
                name=f"foot_{'left' if x_pos < 0.0 else 'right'}_{'front' if y_pos < 0.0 else 'rear'}",
            )

    for side_name, x_pos in (("left", -0.11), ("right", 0.11)):
        face_x = x_pos - 0.024 if x_pos < 0.0 else x_pos + 0.024
        bolt_x = x_pos - 0.028 if x_pos < 0.0 else x_pos + 0.028
        for y_pos in (-0.26, 0.26):
            center_frame.visual(
                Box((0.050, 0.038, 0.080)),
                origin=Origin(xyz=(x_pos, y_pos, 0.800)),
                material=enamel,
                name=f"{side_name}_{'front' if y_pos < 0.0 else 'rear'}_clamp_block",
            )
        hatch_y = -0.26
        center_frame.visual(
            Box((0.004, 0.070, 0.060)),
            origin=Origin(xyz=(face_x, hatch_y, 0.800)),
            material=enamel,
            name=f"{side_name}_service_hatch",
        )
        _add_bolts_on_x_face(
            center_frame,
            x=bolt_x,
            ys=(hatch_y - 0.020, hatch_y + 0.020),
            zs=(0.782, 0.818),
            material=fastener,
            prefix=f"{side_name}_hatch_bolt",
            length=0.010,
        )

    for side_name, x_pos, sign in (
        ("left", -center_width / 2.0 - 0.020, -1.0),
        ("right", center_width / 2.0 + 0.020, 1.0),
    ):
        for bracket_index, y_pos in enumerate((-0.23, 0.23)):
            center_frame.visual(
                Box((0.020, 0.110, 0.028)),
                origin=Origin(xyz=(x_pos + sign * 0.010, y_pos, top_z)),
                material=enamel,
                name=f"{side_name}_hinge_bracket_{bracket_index}",
            )
        center_frame.visual(
            Cylinder(radius=0.009, length=0.22),
            origin=Origin(xyz=(x_pos, -0.23, top_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=enamel,
            name=f"{side_name}_hinge_front_barrel",
        )
        center_frame.visual(
            Cylinder(radius=0.009, length=0.22),
            origin=Origin(xyz=(x_pos, 0.23, top_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=enamel,
            name=f"{side_name}_hinge_rear_barrel",
        )
        center_frame.visual(
            Box((0.012, 0.300, 0.020)),
            origin=Origin(xyz=(x_pos + sign * 0.006, 0.0, top_z - 0.034)),
            material=enamel,
            name=f"{side_name}_stop_pad",
        )
        for y_pos in (-0.135, 0.135):
            center_frame.visual(
                Box((0.018, 0.032, 0.050)),
                origin=Origin(xyz=(x_pos + sign * 0.002, y_pos, top_z - 0.017)),
                material=enamel,
                name=f"{side_name}_{'front' if y_pos < 0.0 else 'rear'}_gusset",
            )

    left_wing = model.part("left_wing")
    left_wing.inertial = Inertial.from_geometry(
        Box((0.32, 0.70, 0.08)),
        mass=2.1,
        origin=Origin(xyz=(-0.16, 0.0, 0.0)),
    )
    _add_wing(
        left_wing,
        mesh_name="left_wing",
        sign=-1.0,
        tube_radius=0.010,
        rack_depth=rack_depth,
        wing_reach=wing_reach,
        frame_material=galvanized,
        adapter_material=enamel,
        fastener_material=fastener,
    )

    right_wing = model.part("right_wing")
    right_wing.inertial = Inertial.from_geometry(
        Box((0.32, 0.70, 0.08)),
        mass=2.1,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
    )
    _add_wing(
        right_wing,
        mesh_name="right_wing",
        sign=1.0,
        tube_radius=0.010,
        rack_depth=rack_depth,
        wing_reach=wing_reach,
        frame_material=galvanized,
        adapter_material=enamel,
        fastener_material=fastener,
    )

    open_limit = math.radians(72.0)
    model.articulation(
        "center_to_left_wing",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=left_wing,
        origin=Origin(xyz=(-center_width / 2.0 - 0.020, 0.0, top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=open_limit,
        ),
    )
    model.articulation(
        "center_to_right_wing",
        ArticulationType.REVOLUTE,
        parent=center_frame,
        child=right_wing,
        origin=Origin(xyz=(center_width / 2.0 + 0.020, 0.0, top_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=open_limit,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_frame = object_model.get_part("center_frame")
    left_wing = object_model.get_part("left_wing")
    right_wing = object_model.get_part("right_wing")
    left_hinge = object_model.get_articulation("center_to_left_wing")
    right_hinge = object_model.get_articulation("center_to_right_wing")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "wing hinge stop ranges are symmetric",
        left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and left_hinge.motion_limits.lower == 0.0
        and right_hinge.motion_limits.lower == 0.0
        and abs(left_hinge.motion_limits.upper - right_hinge.motion_limits.upper) < 1e-9
        and 1.1 < left_hinge.motion_limits.upper < 1.35,
        details="Wing hinges should deploy from the flat working position to a realistic raised stop near 70 degrees.",
    )

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(
            left_wing,
            center_frame,
            elem_a="hinge_barrel",
            elem_b="left_hinge_front_barrel",
            name="left wing remains carried on the hinge line",
        )
        ctx.expect_contact(
            right_wing,
            center_frame,
            elem_a="hinge_barrel",
            elem_b="right_hinge_front_barrel",
            name="right wing remains carried on the hinge line",
        )
        ctx.expect_contact(
            left_wing,
            center_frame,
            elem_a="stop_tab",
            elem_b="left_stop_pad",
            name="left deployed wing lands on a real stop pad",
        )
        ctx.expect_contact(
            right_wing,
            center_frame,
            elem_a="stop_tab",
            elem_b="right_stop_pad",
            name="right deployed wing lands on a real stop pad",
        )

    with ctx.pose({left_hinge: left_hinge.motion_limits.upper, right_hinge: right_hinge.motion_limits.upper}):
        ctx.expect_gap(
            left_wing,
            center_frame,
            axis="z",
            positive_elem="outer_hanging_rail",
            negative_elem="center_mid_rail",
            min_gap=0.18,
            name="left wing outer rail lifts above the center deck at the upper stop",
        )
        ctx.expect_gap(
            right_wing,
            center_frame,
            axis="z",
            positive_elem="outer_hanging_rail",
            negative_elem="center_mid_rail",
            min_gap=0.18,
            name="right wing outer rail lifts above the center deck at the upper stop",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
