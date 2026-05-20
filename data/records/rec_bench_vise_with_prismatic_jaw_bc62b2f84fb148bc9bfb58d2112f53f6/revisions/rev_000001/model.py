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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _jaw_mesh(
    name: str,
    *,
    depth: float,
    width: float,
    height: float,
    notch_depth: float,
    face_positive_x: bool,
):
    half_depth = depth * 0.5
    half_height = height * 0.5
    if face_positive_x:
        profile = [
            (-half_depth, -half_height),
            (half_depth, -half_height),
            (half_depth - notch_depth, 0.0),
            (half_depth, half_height),
            (-half_depth, half_height),
        ]
    else:
        profile = [
            (half_depth, -half_height),
            (-half_depth, -half_height),
            (-half_depth + notch_depth, 0.0),
            (-half_depth, half_height),
            (half_depth, half_height),
        ]
    sections = [
        [(x, -width * 0.5, z) for x, z in profile],
        [(x, width * 0.5, z) for x, z in profile],
    ]
    return _mesh(name, section_loft(sections))


def _tube_shell_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    half_length = length * 0.5
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_length), (outer_radius, half_length)],
        [(inner_radius, -half_length), (inner_radius, half_length)],
        segments=36,
        start_cap="flat",
        end_cap="flat",
    ).rotate_y(math.pi * 0.5)
    return _mesh(name, shell)


def _hex_shaft_mesh(name: str, *, radius: float, length: float):
    return _mesh(
        name,
        CylinderGeometry(radius=radius, height=length, radial_segments=6).rotate_y(math.pi * 0.5),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_drilling_vise")

    cast_iron = model.material("cast_iron", rgba=(0.27, 0.29, 0.30, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.10, 0.10, 0.11, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((0.240, 0.160, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cast_iron,
        name="base_plate",
    )
    base_plate.visual(
        Box((0.180, 0.106, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=dark_steel,
        name="swivel_bearing_land",
    )
    base_plate.inertial = Inertial.from_geometry(
        Box((0.240, 0.160, 0.016)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    turntable = model.part("turntable_plate")
    turntable.visual(
        Cylinder(radius=0.072, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cast_iron,
        name="bearing_ring",
    )
    turntable.visual(
        Box((0.180, 0.120, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=cast_iron,
        name="saddle_plate",
    )
    turntable.visual(
        Box((0.110, 0.092, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=cast_iron,
        name="vise_mount_pad",
    )
    turntable.inertial = Inertial.from_geometry(
        Box((0.180, 0.120, 0.034)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    body = model.part("vise_body")
    body.visual(
        Box((0.154, 0.086, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=cast_iron,
        name="bed_plate",
    )
    body.visual(
        Box((0.130, 0.010, 0.018)),
        origin=Origin(xyz=(0.008, 0.034, 0.014)),
        material=cast_iron,
        name="left_way",
    )
    body.visual(
        Box((0.130, 0.010, 0.018)),
        origin=Origin(xyz=(0.008, -0.034, 0.014)),
        material=cast_iron,
        name="right_way",
    )
    body.visual(
        Box((0.030, 0.086, 0.024)),
        origin=Origin(xyz=(-0.062, 0.0, 0.017)),
        material=cast_iron,
        name="rear_pedestal",
    )
    body.visual(
        Box((0.010, 0.086, 0.034)),
        origin=Origin(xyz=(-0.078, 0.0, 0.022)),
        material=cast_iron,
        name="rear_backstop",
    )
    body.visual(
        Box((0.024, 0.014, 0.040)),
        origin=Origin(xyz=(0.058, 0.033, 0.020)),
        material=cast_iron,
        name="front_left_cheek",
    )
    body.visual(
        Box((0.024, 0.014, 0.040)),
        origin=Origin(xyz=(0.058, -0.033, 0.020)),
        material=cast_iron,
        name="front_right_cheek",
    )
    body.visual(
        Box((0.020, 0.054, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, 0.005)),
        material=cast_iron,
        name="front_lower_tie",
    )
    body.visual(
        Box((0.020, 0.054, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, 0.029)),
        material=cast_iron,
        name="front_upper_tie",
    )
    body.visual(
        _tube_shell_mesh("front_bearing_boss_mesh", outer_radius=0.0115, inner_radius=0.0060, length=0.014),
        origin=Origin(xyz=(0.064, 0.0, 0.017)),
        material=cast_iron,
        name="front_bearing_boss",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.130),
        origin=Origin(xyz=(0.005, 0.022, 0.018), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=machined_steel,
        name="left_guide_rod",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.130),
        origin=Origin(xyz=(0.005, -0.022, 0.018), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=machined_steel,
        name="right_guide_rod",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.164, 0.086, 0.048)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    fixed_jaw = model.part("fixed_rear_jaw")
    fixed_jaw.visual(
        Box((0.030, 0.076, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_steel,
        name="jaw_foot",
    )
    fixed_jaw.visual(
        Box((0.010, 0.082, 0.028)),
        origin=Origin(xyz=(-0.010, 0.0, 0.014)),
        material=black_oxide,
        name="jaw_backer",
    )
    fixed_jaw.visual(
        _jaw_mesh(
            "rear_v_jaw_mesh",
            depth=0.028,
            width=0.072,
            height=0.032,
            notch_depth=0.012,
            face_positive_x=True,
        ),
        origin=Origin(xyz=(0.002, 0.0, 0.026)),
        material=machined_steel,
        name="rear_v_jaw",
    )
    fixed_jaw.inertial = Inertial.from_geometry(
        Box((0.040, 0.082, 0.046)),
        mass=1.3,
        origin=Origin(xyz=(-0.002, 0.0, 0.023)),
    )

    moving_jaw = model.part("moving_front_jaw")
    moving_jaw.visual(
        _tube_shell_mesh("left_sleeve_mesh", outer_radius=0.009, inner_radius=0.0072, length=0.040),
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
        material=black_oxide,
        name="left_sleeve",
    )
    moving_jaw.visual(
        _tube_shell_mesh("right_sleeve_mesh", outer_radius=0.009, inner_radius=0.0072, length=0.040),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=black_oxide,
        name="right_sleeve",
    )
    moving_jaw.visual(
        Box((0.024, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.022, 0.016)),
        material=dark_steel,
        name="left_web",
    )
    moving_jaw.visual(
        Box((0.024, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.022, 0.016)),
        material=dark_steel,
        name="right_web",
    )
    moving_jaw.visual(
        Box((0.020, 0.006, 0.028)),
        origin=Origin(xyz=(0.004, 0.010, 0.009)),
        material=dark_steel,
        name="left_nut_lug",
    )
    moving_jaw.visual(
        Box((0.020, 0.006, 0.028)),
        origin=Origin(xyz=(0.004, -0.010, 0.009)),
        material=dark_steel,
        name="right_nut_lug",
    )
    moving_jaw.visual(
        Box((0.020, 0.020, 0.004)),
        origin=Origin(xyz=(0.004, 0.0, 0.021)),
        material=dark_steel,
        name="nut_bridge",
    )
    moving_jaw.visual(
        _jaw_mesh(
            "front_v_jaw_mesh",
            depth=0.024,
            width=0.072,
            height=0.034,
            notch_depth=0.012,
            face_positive_x=False,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=machined_steel,
        name="front_v_jaw",
    )
    moving_jaw.inertial = Inertial.from_geometry(
        Box((0.040, 0.072, 0.060)),
        mass=1.4,
        origin=Origin(xyz=(0.002, 0.0, 0.026)),
    )

    leadscrew = model.part("leadscrew_handle")
    leadscrew.visual(
        _hex_shaft_mesh("leadscrew_hex_mesh", radius=0.0052, length=0.090),
        origin=Origin(xyz=(0.029, 0.0, 0.0)),
        material=machined_steel,
        name="threaded_shaft",
    )
    leadscrew.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(0.078, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="thrust_collar",
    )
    leadscrew.visual(
        Cylinder(radius=0.0105, length=0.020),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=black_oxide,
        name="handle_hub",
    )
    leadscrew.visual(
        Cylinder(radius=0.0035, length=0.140),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=machined_steel,
        name="t_bar",
    )
    leadscrew.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.090, 0.079, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=black_oxide,
        name="left_grip",
    )
    leadscrew.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.090, -0.079, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=black_oxide,
        name="right_grip",
    )
    leadscrew.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0105, length=0.168),
        mass=0.7,
        origin=Origin(xyz=(0.034, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.5),
    )
    model.articulation(
        "turntable_to_body",
        ArticulationType.FIXED,
        parent=turntable,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
    )
    model.articulation(
        "body_to_fixed_jaw",
        ArticulationType.FIXED,
        parent=body,
        child=fixed_jaw,
        origin=Origin(xyz=(-0.058, 0.0, 0.029)),
    )
    model.articulation(
        "body_to_moving_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=moving_jaw,
        origin=Origin(xyz=(-0.010, 0.0, 0.018)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.048),
    )
    model.articulation(
        "body_to_leadscrew",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=leadscrew,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_plate = object_model.get_part("base_plate")
    turntable = object_model.get_part("turntable_plate")
    body = object_model.get_part("vise_body")
    fixed_jaw = object_model.get_part("fixed_rear_jaw")
    moving_jaw = object_model.get_part("moving_front_jaw")
    leadscrew = object_model.get_part("leadscrew_handle")

    swivel = object_model.get_articulation("base_to_turntable")
    jaw_slide = object_model.get_articulation("body_to_moving_jaw")
    screw_spin = object_model.get_articulation("body_to_leadscrew")

    base_shell = base_plate.get_visual("base_plate")
    bearing_ring = turntable.get_visual("bearing_ring")
    mount_pad = turntable.get_visual("vise_mount_pad")
    bed_plate = body.get_visual("bed_plate")
    rear_pedestal = body.get_visual("rear_pedestal")
    left_guide = body.get_visual("left_guide_rod")
    right_guide = body.get_visual("right_guide_rod")
    jaw_foot = fixed_jaw.get_visual("jaw_foot")
    rear_v = fixed_jaw.get_visual("rear_v_jaw")
    front_v = moving_jaw.get_visual("front_v_jaw")
    front_bearing_boss = body.get_visual("front_bearing_boss")
    left_sleeve = moving_jaw.get_visual("left_sleeve")
    right_sleeve = moving_jaw.get_visual("right_sleeve")
    left_nut_lug = moving_jaw.get_visual("left_nut_lug")
    right_nut_lug = moving_jaw.get_visual("right_nut_lug")
    threaded_shaft = leadscrew.get_visual("threaded_shaft")
    thrust_collar = leadscrew.get_visual("thrust_collar")

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
        "turntable axis is vertical",
        tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"axis={swivel.axis}",
    )
    ctx.check(
        "jaw and screw run along the vise length",
        tuple(jaw_slide.axis) == (1.0, 0.0, 0.0) and tuple(screw_spin.axis) == (1.0, 0.0, 0.0),
        details=f"jaw_axis={jaw_slide.axis}, screw_axis={screw_spin.axis}",
    )

    ctx.expect_contact(
        turntable,
        base_plate,
        elem_a=bearing_ring,
        elem_b=base_shell,
        name="turntable bearing ring sits on the base plate",
    )
    ctx.expect_contact(
        body,
        turntable,
        elem_a=bed_plate,
        elem_b=mount_pad,
        name="vise body seats on the turntable pad",
    )
    ctx.expect_contact(
        fixed_jaw,
        body,
        elem_a=jaw_foot,
        elem_b=rear_pedestal,
        name="fixed rear jaw is mounted to the body pedestal",
    )
    ctx.expect_contact(
        leadscrew,
        body,
        elem_a=thrust_collar,
        elem_b=front_bearing_boss,
        name="leadscrew thrust collar bears on the front bearing boss",
    )

    with ctx.pose({jaw_slide: 0.0}):
        ctx.expect_gap(
            moving_jaw,
            fixed_jaw,
            axis="x",
            positive_elem=front_v,
            negative_elem=rear_v,
            min_gap=0.018,
            max_gap=0.023,
            name="rest pose leaves a narrow jaw opening",
        )
        ctx.expect_within(
            body,
            moving_jaw,
            axes="yz",
            inner_elem=left_guide,
            outer_elem=left_sleeve,
            margin=0.001,
            name="left sleeve stays concentric around the left guide at rest",
        )
        ctx.expect_within(
            body,
            moving_jaw,
            axes="yz",
            inner_elem=right_guide,
            outer_elem=right_sleeve,
            margin=0.001,
            name="right sleeve stays concentric around the right guide at rest",
        )
        ctx.expect_overlap(
            body,
            moving_jaw,
            axes="x",
            elem_a=left_guide,
            elem_b=left_sleeve,
            min_overlap=0.038,
            name="left guide remains captured by the moving jaw at rest",
        )
        ctx.expect_overlap(
            body,
            moving_jaw,
            axes="x",
            elem_a=right_guide,
            elem_b=right_sleeve,
            min_overlap=0.038,
            name="right guide remains captured by the moving jaw at rest",
        )
        ctx.expect_gap(
            leadscrew,
            body,
            axis="z",
            positive_elem=threaded_shaft,
            negative_elem=bed_plate,
            min_gap=0.0015,
            max_gap=0.003,
            name="threaded shaft runs just above the vise bed",
        )
        ctx.expect_gap(
            moving_jaw,
            leadscrew,
            axis="y",
            positive_elem=left_nut_lug,
            negative_elem=threaded_shaft,
            min_gap=0.001,
            max_gap=0.004,
            name="left nut lug clears the threaded shaft",
        )
        ctx.expect_gap(
            leadscrew,
            moving_jaw,
            axis="y",
            positive_elem=threaded_shaft,
            negative_elem=right_nut_lug,
            min_gap=0.001,
            max_gap=0.004,
            name="right nut lug clears the threaded shaft",
        )

    jaw_upper = jaw_slide.motion_limits.upper if jaw_slide.motion_limits is not None else 0.0
    rest_pos = ctx.part_world_position(moving_jaw)
    with ctx.pose({jaw_slide: jaw_upper}):
        ctx.expect_gap(
            moving_jaw,
            fixed_jaw,
            axis="x",
            positive_elem=front_v,
            negative_elem=rear_v,
            min_gap=0.066,
            max_gap=0.071,
            name="front jaw opens forward along the guides",
        )
        ctx.expect_within(
            body,
            moving_jaw,
            axes="yz",
            inner_elem=left_guide,
            outer_elem=left_sleeve,
            margin=0.001,
            name="left sleeve stays concentric around the left guide when opened",
        )
        ctx.expect_within(
            body,
            moving_jaw,
            axes="yz",
            inner_elem=right_guide,
            outer_elem=right_sleeve,
            margin=0.001,
            name="right sleeve stays concentric around the right guide when opened",
        )
        ctx.expect_overlap(
            body,
            moving_jaw,
            axes="x",
            elem_a=left_guide,
            elem_b=left_sleeve,
            min_overlap=0.038,
            name="left guide retains insertion at full opening",
        )
        ctx.expect_overlap(
            body,
            moving_jaw,
            axes="x",
            elem_a=right_guide,
            elem_b=right_sleeve,
            min_overlap=0.038,
            name="right guide retains insertion at full opening",
        )
        extended_pos = ctx.part_world_position(moving_jaw)

    ctx.check(
        "moving jaw translates toward the front stop",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.043,
        details=f"rest={rest_pos}, opened={extended_pos}",
    )

    rest_fixed = ctx.part_world_position(fixed_jaw)
    with ctx.pose({swivel: math.pi * 0.5}):
        swiveled_fixed = ctx.part_world_position(fixed_jaw)
    ctx.check(
        "turntable swings the vise around the vertical axis",
        rest_fixed is not None
        and swiveled_fixed is not None
        and abs(swiveled_fixed[0]) < 0.010
        and swiveled_fixed[1] < -0.050,
        details=f"rest={rest_fixed}, swiveled={swiveled_fixed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
