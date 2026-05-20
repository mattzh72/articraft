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
)


def _add_y_bolt(
    part,
    xyz,
    *,
    radius: float,
    length: float,
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_corner_bolts(
    part,
    *,
    center_x: float,
    center_z: float,
    span_x: float,
    span_z: float,
    y: float,
    radius: float,
    length: float,
    material,
    prefix: str,
) -> None:
    offsets = (
        (-span_x / 2.0, -span_z / 2.0),
        (-span_x / 2.0, span_z / 2.0),
        (span_x / 2.0, -span_z / 2.0),
        (span_x / 2.0, span_z / 2.0),
    )
    for index, (dx, dz) in enumerate(offsets, start=1):
        _add_y_bolt(
            part,
            (center_x + dx, y, center_z + dz),
            radius=radius,
            length=length,
            material=material,
            name=f"{prefix}_bolt_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="legacy_service_access_panel")

    frame_paint = model.material("frame_paint", rgba=(0.34, 0.37, 0.36, 1.0))
    door_paint = model.material("door_paint", rgba=(0.44, 0.46, 0.42, 1.0))
    adapter_paint = model.material("adapter_paint", rgba=(0.54, 0.51, 0.44, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    hardware_bright = model.material("hardware_bright", rgba=(0.70, 0.72, 0.74, 1.0))

    outer_w = 0.68
    outer_h = 0.98
    opening_w = 0.52
    opening_h = 0.82
    jamb_w = (outer_w - opening_w) / 2.0
    rail_h = (outer_h - opening_h) / 2.0

    front_flange_t = 0.008
    sleeve_depth = 0.050

    door_w = 0.580
    door_h = 0.880
    door_t = 0.028
    hinge_axis_x = -0.298
    door_closed_center_y = front_flange_t + door_t / 2.0
    door_body_center_x = 0.298

    frame = model.part("frame")
    frame.visual(
        Box((jamb_w, front_flange_t, outer_h)),
        origin=Origin(xyz=(-outer_w / 2.0 + jamb_w / 2.0, front_flange_t / 2.0, 0.0)),
        material=frame_paint,
        name="left_front_jamb",
    )
    frame.visual(
        Box((jamb_w, front_flange_t, outer_h)),
        origin=Origin(xyz=(outer_w / 2.0 - jamb_w / 2.0, front_flange_t / 2.0, 0.0)),
        material=frame_paint,
        name="right_front_jamb",
    )
    frame.visual(
        Box((outer_w, front_flange_t, rail_h)),
        origin=Origin(xyz=(0.0, front_flange_t / 2.0, outer_h / 2.0 - rail_h / 2.0)),
        material=frame_paint,
        name="front_header",
    )
    frame.visual(
        Box((outer_w, front_flange_t, rail_h)),
        origin=Origin(xyz=(0.0, front_flange_t / 2.0, -outer_h / 2.0 + rail_h / 2.0)),
        material=frame_paint,
        name="front_sill",
    )

    frame.visual(
        Box((jamb_w, sleeve_depth, opening_h)),
        origin=Origin(
            xyz=(-outer_w / 2.0 + jamb_w / 2.0, -sleeve_depth / 2.0, 0.0)
        ),
        material=frame_paint,
        name="left_sleeve",
    )
    frame.visual(
        Box((jamb_w, sleeve_depth, opening_h)),
        origin=Origin(
            xyz=(outer_w / 2.0 - jamb_w / 2.0, -sleeve_depth / 2.0, 0.0)
        ),
        material=frame_paint,
        name="right_sleeve",
    )
    frame.visual(
        Box((opening_w, sleeve_depth, rail_h)),
        origin=Origin(xyz=(0.0, -sleeve_depth / 2.0, outer_h / 2.0 - rail_h / 2.0)),
        material=frame_paint,
        name="rear_header",
    )
    frame.visual(
        Box((opening_w, sleeve_depth, rail_h)),
        origin=Origin(xyz=(0.0, -sleeve_depth / 2.0, -outer_h / 2.0 + rail_h / 2.0)),
        material=frame_paint,
        name="rear_sill",
    )

    frame.visual(
        Box((0.060, 0.006, 0.780)),
        origin=Origin(xyz=(-0.320, 0.011, 0.0)),
        material=adapter_paint,
        name="hinge_doubler",
    )
    frame.visual(
        Box((0.060, 0.006, 0.780)),
        origin=Origin(xyz=(0.320, 0.011, 0.0)),
        material=adapter_paint,
        name="latch_doubler",
    )
    frame.visual(
        Box((0.022, 0.008, 0.780)),
        origin=Origin(xyz=(-0.324, 0.018, 0.0)),
        material=hardware_dark,
        name="frame_hinge_leaf",
    )

    for index, z in enumerate((0.290, 0.000, -0.290), start=1):
        frame.visual(
            Cylinder(radius=0.012, length=0.130),
            origin=Origin(xyz=(hinge_axis_x, door_closed_center_y, z)),
            material=hardware_dark,
            name=f"frame_hinge_barrel_{index}",
        )

    frame.visual(
        Box((0.032, 0.018, 0.085)),
        origin=Origin(xyz=(0.318, 0.020, 0.220)),
        material=hardware_dark,
        name="top_keeper",
    )
    frame.visual(
        Box((0.032, 0.018, 0.085)),
        origin=Origin(xyz=(0.318, 0.020, -0.220)),
        material=hardware_dark,
        name="bottom_keeper",
    )

    _add_corner_bolts(
        frame,
        center_x=-0.320,
        center_z=0.0,
        span_x=0.028,
        span_z=0.600,
        y=0.016,
        radius=0.0055,
        length=0.004,
        material=hardware_bright,
        prefix="hinge_doubler",
    )
    _add_corner_bolts(
        frame,
        center_x=0.320,
        center_z=0.0,
        span_x=0.028,
        span_z=0.600,
        y=0.016,
        radius=0.0055,
        length=0.004,
        material=hardware_bright,
        prefix="latch_doubler",
    )

    frame.inertial = Inertial.from_geometry(
        Box((outer_w, 0.085, outer_h)),
        mass=18.0,
        origin=Origin(xyz=(0.0, -0.0075, 0.0)),
    )

    door = model.part("service_panel")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_body_center_x, 0.0, 0.0)),
        material=door_paint,
        name="door_shell",
    )

    door.visual(
        Box((0.060, 0.010, 0.840)),
        origin=Origin(xyz=(0.038, -0.009, 0.0)),
        material=hardware_dark,
        name="hinge_stile_reinforcement",
    )
    door.visual(
        Box((0.052, 0.010, 0.720)),
        origin=Origin(xyz=(0.554, -0.009, 0.0)),
        material=hardware_dark,
        name="latch_stile_reinforcement",
    )
    door.visual(
        Box((0.360, 0.010, 0.055)),
        origin=Origin(xyz=(0.290, -0.009, 0.0)),
        material=hardware_dark,
        name="mid_stiffener",
    )

    door.visual(
        Box((0.036, 0.008, 0.780)),
        origin=Origin(xyz=(0.026, 0.018, 0.0)),
        material=hardware_dark,
        name="door_hinge_leaf",
    )
    for index, z in enumerate((0.145, -0.145), start=1):
        door.visual(
            Cylinder(radius=0.012, length=0.130),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hardware_dark,
            name=f"door_hinge_barrel_{index}",
        )

    door.visual(
        Box((0.070, 0.008, 0.700)),
        origin=Origin(xyz=(0.542, 0.018, 0.0)),
        material=adapter_paint,
        name="latch_face_plate",
    )
    door.visual(
        Box((0.014, 0.010, 0.620)),
        origin=Origin(xyz=(0.548, 0.023, 0.0)),
        material=hardware_dark,
        name="latch_rod",
    )
    door.visual(
        Box((0.090, 0.010, 0.120)),
        origin=Origin(xyz=(0.500, 0.023, 0.0)),
        material=hardware_dark,
        name="latch_handle_base",
    )
    door.visual(
        Box((0.020, 0.054, 0.120)),
        origin=Origin(xyz=(0.468, 0.041, 0.0)),
        material=hardware_dark,
        name="latch_handle_grip",
    )
    door.visual(
        Box((0.020, 0.016, 0.060)),
        origin=Origin(xyz=(0.578, 0.018, 0.220)),
        material=hardware_dark,
        name="upper_latch_dog",
    )
    door.visual(
        Box((0.020, 0.016, 0.060)),
        origin=Origin(xyz=(0.578, 0.018, -0.220)),
        material=hardware_dark,
        name="lower_latch_dog",
    )

    door.visual(
        Box((0.230, 0.006, 0.180)),
        origin=Origin(xyz=(0.250, 0.017, 0.225)),
        material=adapter_paint,
        name="upper_service_adapter",
    )
    door.visual(
        Box((0.190, 0.006, 0.140)),
        origin=Origin(xyz=(0.250, 0.023, 0.225)),
        material=door_paint,
        name="upper_service_hatch_cover",
    )
    _add_corner_bolts(
        door,
        center_x=0.250,
        center_z=0.225,
        span_x=0.170,
        span_z=0.120,
        y=0.020,
        radius=0.0045,
        length=0.004,
        material=hardware_bright,
        prefix="upper_service_hatch",
    )

    door.visual(
        Box((0.200, 0.006, 0.170)),
        origin=Origin(xyz=(0.315, 0.017, -0.235)),
        material=adapter_paint,
        name="lower_service_adapter",
    )
    door.visual(
        Box((0.160, 0.006, 0.130)),
        origin=Origin(xyz=(0.315, 0.023, -0.235)),
        material=door_paint,
        name="lower_service_hatch_cover",
    )
    _add_corner_bolts(
        door,
        center_x=0.315,
        center_z=-0.235,
        span_x=0.140,
        span_z=0.110,
        y=0.020,
        radius=0.0045,
        length=0.004,
        material=hardware_bright,
        prefix="lower_service_hatch",
    )

    _add_corner_bolts(
        door,
        center_x=0.542,
        center_z=0.0,
        span_x=0.036,
        span_z=0.520,
        y=0.022,
        radius=0.0045,
        length=0.004,
        material=hardware_bright,
        prefix="latch_face_plate",
    )

    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=11.0,
        origin=Origin(xyz=(door_body_center_x, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, door_closed_center_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    door = object_model.get_part("service_panel")
    hinge = object_model.get_articulation("frame_to_service_panel")

    # Resolve prompt-critical named visuals so missing hardware becomes a hard failure.
    frame.get_visual("top_keeper")
    frame.get_visual("bottom_keeper")
    door.get_visual("door_shell")
    door.get_visual("upper_service_hatch_cover")
    door.get_visual("lower_service_hatch_cover")
    door.get_visual("upper_latch_dog")

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
        "hinge_axis_is_vertical",
        tuple(hinge.axis) == (0.0, 0.0, 1.0),
        f"Expected a vertical hinge axis, got {hinge.axis!r}.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            door,
            frame,
            name="door_seats_on_frame",
        )
        ctx.expect_overlap(
            door,
            frame,
            axes="xz",
            min_overlap=0.50,
            name="door_covers_framed_opening",
        )
        ctx.expect_gap(
            frame,
            door,
            axis="x",
            positive_elem="top_keeper",
            negative_elem="upper_latch_dog",
            min_gap=0.008,
            max_gap=0.020,
            name="latch_hardware_closes_against_keeper_side",
        )

    closed_shell = ctx.part_element_world_aabb(door, elem="door_shell")
    with ctx.pose({hinge: 1.10}):
        open_shell = ctx.part_element_world_aabb(door, elem="door_shell")
    if closed_shell is not None and open_shell is not None:
        closed_center_y = 0.5 * (closed_shell[0][1] + closed_shell[1][1])
        open_center_y = 0.5 * (open_shell[0][1] + open_shell[1][1])
        ctx.check(
            "door_swings_outward",
            open_center_y > closed_center_y + 0.18,
            (
                "Door did not swing clearly outward: "
                f"closed_center_y={closed_center_y:.4f}, open_center_y={open_center_y:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
