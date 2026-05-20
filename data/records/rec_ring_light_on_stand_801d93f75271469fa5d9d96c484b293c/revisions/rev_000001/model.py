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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


CASTER_COUNT = 5
CASTER_RADIUS = 0.56


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_column_sleeve_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.052, 0.11),
            (0.059, 0.135),
            (0.057, 0.60),
            (0.048, 0.66),
        ],
        [
            (0.041, 0.11),
            (0.047, 0.135),
            (0.045, 0.60),
            (0.039, 0.66),
        ],
        segments=52,
    )


def _build_leg_mesh():
    return tube_from_spline_points(
        [
            (0.10, 0.0, 0.088),
            (0.20, 0.0, 0.092),
            (0.36, 0.0, 0.090),
            (0.52, 0.0, 0.086),
        ],
        radius=0.019,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )


def _build_ring_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.154, -0.030),
            (0.188, -0.034),
            (0.223, -0.028),
            (0.232, -0.010),
            (0.228, 0.012),
            (0.214, 0.024),
            (0.168, 0.028),
        ],
        [
            (0.148, -0.020),
            (0.176, -0.024),
            (0.206, -0.019),
            (0.214, -0.008),
            (0.210, 0.010),
            (0.194, 0.020),
            (0.154, 0.022),
        ],
        segments=72,
    )


def _build_ring_diffuser_mesh():
    return LatheGeometry(
        [
            (0.154, 0.0215),
            (0.225, 0.0215),
            (0.225, 0.0265),
            (0.154, 0.0265),
            (0.154, 0.0215),
        ],
        segments=72,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_ring_light")

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    aluminum = model.material("aluminum", rgba=(0.75, 0.77, 0.80, 1.0))
    light_diffuser = model.material("light_diffuser", rgba=(0.97, 0.97, 0.95, 0.92))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Cylinder(radius=0.175, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_steel,
        name="ballast_skirt",
    )
    base_frame.visual(
        Cylinder(radius=0.135, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=matte_black,
        name="hub_drum",
    )
    base_frame.visual(
        _mesh("column_sleeve", _build_column_sleeve_mesh()),
        material=satin_black,
        name="column_sleeve",
    )

    leg_mesh = _mesh("caster_leg", _build_leg_mesh())
    socket_radius = 0.515
    for index in range(CASTER_COUNT):
        angle = index * math.tau / CASTER_COUNT
        x = socket_radius * math.cos(angle)
        y = socket_radius * math.sin(angle)
        base_frame.visual(
            leg_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=satin_black,
            name=f"leg_{index}",
        )
        base_frame.visual(
            Box((0.055, 0.040, 0.016)),
            origin=Origin(xyz=(x, y, 0.090), rpy=(0.0, 0.0, angle)),
            material=dark_steel,
            name=f"caster_socket_{index}",
        )
    base_frame.inertial = Inertial.from_geometry(
        Box((1.25, 1.25, 0.66)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.33)),
    )

    upper_column = model.part("upper_column")
    upper_column.visual(
        Cylinder(radius=0.033, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=aluminum,
        name="guide_insert",
    )
    upper_column.visual(
        Cylinder(radius=0.038, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
        material=aluminum,
        name="visible_column",
    )
    upper_column.visual(
        Cylinder(radius=0.058, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.559)),
        material=dark_steel,
        name="stop_collar",
    )
    upper_column.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 1.45)),
        mass=5.6,
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
    )

    tilt_bracket = model.part("tilt_bracket")
    tilt_bracket.visual(
        Box((0.070, 0.050, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_steel,
        name="mount_block",
    )
    tilt_bracket.visual(
        Box((0.050, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, -0.020, 0.055)),
        material=dark_steel,
        name="mount_saddle",
    )
    tilt_bracket.visual(
        Box((0.380, 0.022, 0.028)),
        origin=Origin(xyz=(0.0, -0.051, 0.086)),
        material=dark_steel,
        name="rear_stem",
    )
    tilt_bracket.visual(
        Box((0.480, 0.022, 0.040)),
        origin=Origin(xyz=(0.0, -0.050, 0.310)),
        material=dark_steel,
        name="pivot_housing",
    )
    tilt_bracket.visual(
        Box((0.024, 0.028, 0.240)),
        origin=Origin(xyz=(-0.175, -0.070, 0.220)),
        material=dark_steel,
        name="rear_yoke_beam",
    )
    tilt_bracket.visual(
        Box((0.024, 0.028, 0.240)),
        origin=Origin(xyz=(0.175, -0.070, 0.220)),
        material=dark_steel,
        name="right_support_strut",
    )
    tilt_bracket.visual(
        Box((0.022, 0.056, 0.150)),
        origin=Origin(xyz=(-0.251, -0.017, 0.310)),
        material=dark_steel,
        name="left_cheek",
    )
    tilt_bracket.visual(
        Box((0.022, 0.056, 0.150)),
        origin=Origin(xyz=(0.251, -0.017, 0.310)),
        material=dark_steel,
        name="right_cheek",
    )
    tilt_bracket.visual(
        Box((0.038, 0.024, 0.050)),
        origin=Origin(xyz=(-0.242, -0.050, 0.392)),
        material=dark_steel,
        name="left_clip_cap",
    )
    tilt_bracket.visual(
        Box((0.038, 0.024, 0.050)),
        origin=Origin(xyz=(0.242, -0.050, 0.392)),
        material=dark_steel,
        name="right_clip_cap",
    )
    tilt_bracket.visual(
        Box((0.026, 0.040, 0.060)),
        origin=Origin(xyz=(0.262, -0.012, 0.310)),
        material=dark_steel,
        name="knob_boss",
    )
    tilt_bracket.inertial = Inertial.from_geometry(
        Box((0.60, 0.12, 0.44)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -0.04, 0.22)),
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        _mesh("ring_housing", _build_ring_shell_mesh()),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="ring_housing",
    )
    ring_head.visual(
        _mesh("ring_diffuser", _build_ring_diffuser_mesh()),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_diffuser,
        name="ring_diffuser",
    )
    ring_head.visual(
        Box((0.300, 0.056, 0.060)),
        origin=Origin(xyz=(0.0, -0.058, -0.170)),
        material=satin_black,
        name="rear_driver_box",
    )
    ring_head.visual(
        Box((0.070, 0.032, 0.150)),
        origin=Origin(xyz=(0.0, -0.036, -0.100)),
        material=satin_black,
        name="driver_bridge",
    )
    ring_head.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(-0.215, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_trunnion",
    )
    ring_head.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.215, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_trunnion",
    )
    ring_head.inertial = Inertial.from_geometry(
        Box((0.48, 0.09, 0.52)),
        mass=2.8,
        origin=Origin(xyz=(0.0, -0.01, 0.0)),
    )

    adjustment_knob = model.part("adjustment_knob")
    adjustment_knob.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="knob_shaft",
    )
    adjustment_knob.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="knob_body",
    )
    adjustment_knob.visual(
        Box((0.016, 0.014, 0.032)),
        origin=Origin(xyz=(0.030, 0.020, 0.0)),
        material=matte_black,
        name="knob_handle",
    )
    adjustment_knob.inertial = Inertial.from_geometry(
        Box((0.05, 0.05, 0.05)),
        mass=0.08,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    for index in range(CASTER_COUNT):
        swivel = model.part(f"caster_swivel_{index}")
        swivel.visual(
            Cylinder(radius=0.010, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=dark_steel,
            name="stem",
        )
        swivel.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=dark_steel,
            name="barrel",
        )
        swivel.visual(
            Box((0.022, 0.050, 0.010)),
            origin=Origin(xyz=(0.0, -0.015, -0.018)),
            material=dark_steel,
            name="trail_arm",
        )
        swivel.visual(
            Box((0.004, 0.018, 0.050)),
            origin=Origin(xyz=(-0.016, -0.032, -0.046)),
            material=dark_steel,
            name="left_fork_plate",
        )
        swivel.visual(
            Box((0.004, 0.018, 0.050)),
            origin=Origin(xyz=(0.016, -0.032, -0.046)),
            material=dark_steel,
            name="right_fork_plate",
        )
        swivel.visual(
            Box((0.034, 0.010, 0.010)),
            origin=Origin(xyz=(0.0, -0.040, -0.018)),
            material=dark_steel,
            name="fork_crown",
        )
        swivel.inertial = Inertial.from_geometry(
            Box((0.05, 0.07, 0.09)),
            mass=0.20,
            origin=Origin(xyz=(0.0, -0.015, -0.040)),
        )

        wheel = model.part(f"caster_wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.027, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.004, length=0.006),
            origin=Origin(xyz=(-0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name="left_axle_stub",
        )
        wheel.visual(
            Cylinder(radius=0.004, length=0.006),
            origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name="right_axle_stub",
        )
        wheel.visual(
            Cylinder(radius=0.002, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.024)),
            material=dark_steel,
            name="valve_stem",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.027, length=0.018),
            mass=0.16,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

    model.articulation(
        "base_to_column",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.30,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "column_to_bracket",
        ArticulationType.FIXED,
        parent=upper_column,
        child=tilt_bracket,
        origin=Origin(xyz=(0.0, 0.0, 1.43)),
    )
    model.articulation(
        "bracket_to_ring",
        ArticulationType.REVOLUTE,
        parent=tilt_bracket,
        child=ring_head,
        origin=Origin(xyz=(0.0, 0.0, 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.4,
            lower=-0.95,
            upper=0.95,
        ),
    )
    model.articulation(
        "bracket_to_knob",
        ArticulationType.CONTINUOUS,
        parent=tilt_bracket,
        child=adjustment_knob,
        origin=Origin(xyz=(0.275, 0.0, 0.31)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=6.0,
        ),
    )

    for index in range(CASTER_COUNT):
        angle = index * math.tau / CASTER_COUNT
        x = CASTER_RADIUS * math.cos(angle)
        y = CASTER_RADIUS * math.sin(angle)
        swivel = model.get_part(f"caster_swivel_{index}")
        wheel = model.get_part(f"caster_wheel_{index}")
        model.articulation(
            f"base_to_caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base_frame,
            child=swivel,
            origin=Origin(xyz=(x, y, 0.082)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=10.0,
            ),
        )
        model.articulation(
            f"caster_swivel_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=swivel,
            child=wheel,
            origin=Origin(xyz=(0.0, -0.034, -0.052)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=30.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    upper_column = object_model.get_part("upper_column")
    tilt_bracket = object_model.get_part("tilt_bracket")
    ring_head = object_model.get_part("ring_head")
    adjustment_knob = object_model.get_part("adjustment_knob")

    column_slide = object_model.get_articulation("base_to_column")
    ring_tilt = object_model.get_articulation("bracket_to_ring")
    knob_turn = object_model.get_articulation("bracket_to_knob")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(upper_column, base_frame, contact_tol=0.0015, name="column seats on sleeve")
    ctx.expect_contact(tilt_bracket, upper_column, contact_tol=0.0015, name="bracket seats on column")
    ctx.expect_contact(ring_head, tilt_bracket, contact_tol=0.0015, name="ring head clipped in bracket")
    ctx.expect_contact(
        adjustment_knob,
        tilt_bracket,
        contact_tol=0.0015,
        name="side knob seated on bracket",
    )

    ctx.check(
        "column slide axis is vertical",
        tuple(column_slide.axis) == (0.0, 0.0, 1.0),
        details=f"axis={column_slide.axis}",
    )
    ctx.check(
        "ring tilt axis is horizontal",
        tuple(ring_tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={ring_tilt.axis}",
    )
    ctx.check(
        "knob axis follows tilt axle",
        tuple(knob_turn.axis) == (1.0, 0.0, 0.0),
        details=f"axis={knob_turn.axis}",
    )

    for index in range(CASTER_COUNT):
        swivel = object_model.get_part(f"caster_swivel_{index}")
        wheel = object_model.get_part(f"caster_wheel_{index}")
        swivel_joint = object_model.get_articulation(f"base_to_caster_swivel_{index}")
        wheel_joint = object_model.get_articulation(f"caster_swivel_to_wheel_{index}")

        ctx.expect_contact(
            swivel,
            base_frame,
            contact_tol=0.0015,
            name=f"caster swivel {index} mounted to base",
        )
        ctx.expect_contact(
            wheel,
            swivel,
            contact_tol=0.0015,
            name=f"caster wheel {index} mounted in fork",
        )
        ctx.check(
            f"caster swivel {index} turns about vertical axis",
            tuple(swivel_joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis={swivel_joint.axis}",
        )
        ctx.check(
            f"caster wheel {index} spins on horizontal axle",
            tuple(wheel_joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={wheel_joint.axis}",
        )

    bracket_rest = ctx.part_world_position(tilt_bracket)
    assert bracket_rest is not None
    with ctx.pose({column_slide: 0.26}):
        bracket_raised = ctx.part_world_position(tilt_bracket)
        assert bracket_raised is not None
        ctx.check(
            "column extension raises the light head assembly",
            bracket_raised[2] > bracket_rest[2] + 0.24,
            details=f"rest_z={bracket_rest[2]:.4f}, raised_z={bracket_raised[2]:.4f}",
        )

    driver_rest = ctx.part_element_world_aabb(ring_head, elem="rear_driver_box")
    assert driver_rest is not None
    with ctx.pose({ring_tilt: 0.75}):
        driver_tilted = ctx.part_element_world_aabb(ring_head, elem="rear_driver_box")
        assert driver_tilted is not None
        ctx.check(
            "tilt joint moves the ring head about its side axis",
            driver_tilted[0][1] > driver_rest[1][1] + 0.04,
            details=(
                f"rest_max_y={driver_rest[1][1]:.4f}, "
                f"tilted_min_y={driver_tilted[0][1]:.4f}"
            ),
        )
        ctx.expect_contact(
            ring_head,
            tilt_bracket,
            contact_tol=0.0015,
            name="ring remains supported while tilted",
        )

    handle_rest = ctx.part_element_world_aabb(adjustment_knob, elem="knob_handle")
    assert handle_rest is not None
    with ctx.pose({knob_turn: 1.1}):
        handle_turned = ctx.part_element_world_aabb(adjustment_knob, elem="knob_handle")
        assert handle_turned is not None
        ctx.check(
            "side knob rotates on its local axis",
            handle_turned[1][2] > handle_rest[1][2] + 0.01,
            details=f"rest_top_z={handle_rest[1][2]:.4f}, turned_top_z={handle_turned[1][2]:.4f}",
        )

    wheel_0 = object_model.get_part("caster_wheel_0")
    swivel_0 = object_model.get_articulation("base_to_caster_swivel_0")
    wheel_spin_0 = object_model.get_articulation("caster_swivel_to_wheel_0")

    wheel_rest = ctx.part_world_position(wheel_0)
    assert wheel_rest is not None
    with ctx.pose({swivel_0: math.pi / 2.0}):
        wheel_swiveled = ctx.part_world_position(wheel_0)
        assert wheel_swiveled is not None
        ctx.check(
            "caster swivel offsets the wheel around the stem",
            abs(wheel_swiveled[1] - wheel_rest[1]) > 0.02,
            details=f"rest={wheel_rest}, swiveled={wheel_swiveled}",
        )

    valve_rest = ctx.part_element_world_aabb(wheel_0, elem="valve_stem")
    assert valve_rest is not None
    with ctx.pose({wheel_spin_0: math.pi / 2.0}):
        valve_spun = ctx.part_element_world_aabb(wheel_0, elem="valve_stem")
        assert valve_spun is not None
        ctx.check(
            "caster wheel spin turns the wheel around its axle",
            abs(valve_spun[1][1] - valve_rest[1][1]) > 0.015,
            details=f"rest={valve_rest}, spun={valve_spun}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
