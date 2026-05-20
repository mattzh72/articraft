from __future__ import annotations

from math import pi, cos, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ClevisBracketGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


def _rounded_plate_mesh(width: float, depth: float, height: float, radius: float, name: str):
    """Low-cost molded/extruded rounded rectangular solid, centered in XY/Z."""
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius, corner_segments=8),
            height,
            center=True,
        ),
        name,
    )


def _tube_x_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    """A bored hinge/trunnion barrel with its axis along local X."""
    outer = superellipse_profile(outer_radius * 2.0, outer_radius * 2.0, exponent=2.0, segments=56)
    inner = superellipse_profile(inner_radius * 2.0, inner_radius * 2.0, exponent=2.0, segments=40)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, [inner], length, center=True).rotate_y(pi / 2.0),
        name,
    )


def _dust_cup_shell_mesh(name: str):
    height = 0.170
    wall = 0.003
    outer_radius = 0.043
    outer = [
        (outer_radius * 0.92, 0.000),
        (outer_radius, 0.018),
        (outer_radius * 1.02, height * 0.72),
        (outer_radius * 0.96, height),
    ]
    inner = [
        (outer_radius * 0.92 - wall, 0.010),
        (outer_radius - wall, 0.024),
        (outer_radius * 1.02 - wall, height * 0.70),
        (outer_radius * 0.90 - wall, height - 0.010),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer, inner, segments=48, lip_samples=5),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_folding_stick_vacuum")

    black = model.material("black_molded_plastic", rgba=(0.035, 0.038, 0.040, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    grey = model.material("warm_grey_plastic", rgba=(0.45, 0.47, 0.48, 1.0))
    aluminum = model.material("brushed_aluminum_tube", rgba=(0.72, 0.74, 0.76, 1.0))
    steel = model.material("zinc_plated_pins", rgba=(0.78, 0.78, 0.74, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    blue = model.material("blue_brush_roll", rgba=(0.05, 0.28, 0.68, 1.0))
    clear = model.material("smoke_clear_bin", rgba=(0.55, 0.72, 0.78, 0.38))
    accent = model.material("release_red", rgba=(0.82, 0.06, 0.035, 1.0))

    # Root subassembly: a shallow stamped/molded floor head.  It carries the
    # nozzle pitch yoke, the brush roller, and small molded wheels as one
    # factory module to minimize separate parts.
    floor_head = model.part("floor_head")
    floor_head.visual(
        _rounded_plate_mesh(0.305, 0.110, 0.046, 0.018, "floor_head_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=charcoal,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.292, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, 0.055, 0.027)),
        material=rubber,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.250, 0.074, 0.006)),
        origin=Origin(xyz=(0.0, 0.000, 0.004)),
        material=black,
        name="stamped_sole_plate",
    )
    floor_head.visual(
        Cylinder(radius=0.018, length=0.238),
        origin=Origin(xyz=(0.0, 0.035, 0.019), rpy=(0.0, pi / 2.0, 0.0)),
        material=blue,
        name="brush_roll",
    )
    floor_head.visual(
        Cylinder(radius=0.008, length=0.276),
        origin=Origin(xyz=(0.0, -0.046, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="wheel_axle",
    )
    for x in (-0.105, 0.105):
        floor_head.visual(
            Box((0.030, 0.020, 0.030)),
            origin=Origin(xyz=(x, -0.046, 0.026)),
            material=black,
            name=f"axle_boss_{0 if x < 0 else 1}",
        )
    for x in (-0.148, 0.148):
        floor_head.visual(
            Cylinder(radius=0.020, length=0.018),
            origin=Origin(xyz=(x, -0.046, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
            material=rubber,
            name=f"rear_wheel_{0 if x < 0 else 1}",
        )

    floor_head.visual(
        mesh_from_geometry(
            ClevisBracketGeometry(
                (0.090, 0.036, 0.066),
                gap_width=0.052,
                bore_diameter=0.014,
                bore_center_z=0.038,
                base_thickness=0.016,
                corner_radius=0.004,
                center=False,
            ),
            "nozzle_yoke",
        ),
        origin=Origin(xyz=(0.0, -0.058, 0.052)),
        material=grey,
        name="nozzle_yoke",
    )
    floor_head.visual(
        Cylinder(radius=0.005, length=0.088),
        origin=Origin(xyz=(0.0, -0.058, 0.090), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="nozzle_pin",
    )
    for x in (-0.047, 0.047):
        floor_head.visual(
            Cylinder(radius=0.009, length=0.004),
            origin=Origin(xyz=(0.046 if x > 0 else -0.046, -0.058, 0.090), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"nozzle_pin_head_{0 if x < 0 else 1}",
        )
    for x in (-0.032, 0.032):
        floor_head.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(x, -0.075, 0.066), rpy=(pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"nozzle_bolt_{0 if x < 0 else 1}",
        )

    # Lower wand module: a single extruded tube with a molded overmold at each
    # end.  The lower trunnion sits between the head yoke cheeks; the top clevis
    # makes the folding hinge assembly order obvious.
    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        _tube_x_mesh(0.017, 0.0065, 0.046, "nozzle_trunnion"),
        material=grey,
        name="nozzle_trunnion",
    )
    lower_wand.visual(
        Cylinder(radius=0.023, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=grey,
        name="nozzle_socket",
    )
    lower_wand.visual(
        Cylinder(radius=0.014, length=0.612),
        origin=Origin(xyz=(0.0, 0.0, 0.334)),
        material=aluminum,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.042, 0.026, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=grey,
        name="snap_socket_body",
    )
    lower_wand.visual(
        Box((0.026, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, -0.018, 0.108)),
        material=accent,
        name="nozzle_release_tab",
    )
    lower_wand.visual(
        mesh_from_geometry(
            ClevisBracketGeometry(
                (0.075, 0.034, 0.070),
                gap_width=0.044,
                bore_diameter=0.012,
                bore_center_z=0.045,
                base_thickness=0.016,
                corner_radius=0.004,
                center=False,
            ),
            "fold_clevis",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.615)),
        material=grey,
        name="fold_clevis",
    )
    lower_wand.visual(
        Cylinder(radius=0.004, length=0.073),
        origin=Origin(xyz=(0.0, 0.0, 0.660), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="fold_pin",
    )
    for x in (-0.039, 0.039):
        lower_wand.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(0.038 if x > 0 else -0.038, 0.0, 0.660), rpy=(0.0, pi / 2.0, 0.0)),
            material=steel,
            name=f"fold_pin_head_{0 if x < 0 else 1}",
        )
    lower_wand.visual(
        Box((0.050, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, -0.019, 0.633)),
        material=accent,
        name="fold_release_paddle",
    )

    # Upper powered body: the expensive parts are consolidated into one molded
    # carrier around the hinge barrel, tube, motor pod, bin, battery and handle.
    upper_body = model.part("upper_body")
    upper_body.visual(
        _tube_x_mesh(0.016, 0.0055, 0.038, "fold_barrel"),
        material=grey,
        name="fold_barrel",
    )
    upper_body.visual(
        Cylinder(radius=0.020, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=grey,
        name="fold_clamp_band",
    )
    upper_body.visual(
        Cylinder(radius=0.014, length=0.440),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=aluminum,
        name="upper_tube",
    )
    upper_body.visual(
        _rounded_plate_mesh(0.118, 0.108, 0.130, 0.018, "motor_housing"),
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        material=charcoal,
        name="motor_housing",
    )
    upper_body.visual(
        Cylinder(radius=0.036, length=0.070),
        origin=Origin(xyz=(0.0, -0.004, 0.578), rpy=(pi / 2.0, 0.0, 0.0)),
        material=black,
        name="filter_cap",
    )
    upper_body.visual(
        _dust_cup_shell_mesh("transparent_dust_cup"),
        origin=Origin(xyz=(0.0, 0.060, 0.375)),
        material=clear,
        name="dust_cup_shell",
    )
    upper_body.visual(
        Box((0.070, 0.030, 0.054)),
        origin=Origin(xyz=(0.0, 0.040, 0.468)),
        material=grey,
        name="bin_latch_neck",
    )
    upper_body.visual(
        Box((0.086, 0.052, 0.070)),
        origin=Origin(xyz=(0.0, -0.030, 0.410)),
        material=black,
        name="battery_pack",
    )
    upper_body.visual(
        Box((0.070, 0.038, 0.184)),
        origin=Origin(xyz=(0.0, -0.055, 0.590)),
        material=charcoal,
        name="handle_mount",
    )
    upper_body.visual(
        Cylinder(radius=0.013, length=0.222),
        origin=Origin(xyz=(0.0, -0.128, 0.592)),
        material=rubber,
        name="handle_grip",
    )
    upper_body.visual(
        Box((0.036, 0.076, 0.026)),
        origin=Origin(xyz=(0.0, -0.095, 0.694)),
        material=charcoal,
        name="upper_handle_bridge",
    )
    upper_body.visual(
        Box((0.036, 0.076, 0.026)),
        origin=Origin(xyz=(0.0, -0.095, 0.505)),
        material=charcoal,
        name="lower_handle_bridge",
    )
    upper_body.visual(
        Box((0.030, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.079, 0.621)),
        material=grey,
        name="trigger_boss",
    )
    upper_body.visual(
        Box((0.035, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.087, 0.530)),
        material=accent,
        name="bin_release_button",
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=accent,
        name="trigger_pivot_barrel",
    )
    trigger.visual(
        Box((0.020, 0.012, 0.062)),
        origin=Origin(xyz=(0.0, -0.006, -0.036)),
        material=accent,
        name="trigger_lever",
    )

    model.articulation(
        "nozzle_pitch",
        ArticulationType.REVOLUTE,
        parent=floor_head,
        child=lower_wand,
        origin=Origin(xyz=(0.0, -0.058, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.2, lower=-0.65, upper=0.82),
    )
    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=upper_body,
        origin=Origin(xyz=(0.0, 0.0, 0.660)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.2, lower=0.0, upper=2.15),
    )
    model.articulation(
        "trigger_pivot",
        ArticulationType.REVOLUTE,
        parent=upper_body,
        child=trigger,
        origin=Origin(xyz=(0.0, -0.079, 0.621)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=0.0, upper=0.34),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    floor_head = object_model.get_part("floor_head")
    lower_wand = object_model.get_part("lower_wand")
    upper_body = object_model.get_part("upper_body")
    trigger = object_model.get_part("trigger")

    nozzle = object_model.get_articulation("nozzle_pitch")
    fold = object_model.get_articulation("fold_hinge")
    trigger_joint = object_model.get_articulation("trigger_pivot")

    ctx.allow_overlap(
        floor_head,
        lower_wand,
        elem_a="nozzle_pin",
        elem_b="nozzle_trunnion",
        reason="The steel nozzle pin is intentionally captured through the molded trunnion barrel.",
    )
    ctx.allow_overlap(
        lower_wand,
        upper_body,
        elem_a="fold_pin",
        elem_b="fold_barrel",
        reason="The fold hinge pin is intentionally seated through the upper barrel bore.",
    )
    ctx.allow_overlap(
        trigger,
        upper_body,
        elem_a="trigger_pivot_barrel",
        elem_b="trigger_boss",
        reason="The trigger pivot barrel is intentionally molded into the handle boss.",
    )

    ctx.expect_overlap(
        lower_wand,
        floor_head,
        axes="x",
        elem_a="nozzle_trunnion",
        elem_b="nozzle_yoke",
        min_overlap=0.040,
        name="nozzle trunnion is captured between yoke cheeks",
    )
    ctx.expect_overlap(
        upper_body,
        lower_wand,
        axes="x",
        elem_a="fold_barrel",
        elem_b="fold_clevis",
        min_overlap=0.032,
        name="fold barrel is captured between clevis cheeks",
    )
    ctx.expect_overlap(
        trigger,
        upper_body,
        axes="x",
        elem_a="trigger_pivot_barrel",
        elem_b="trigger_boss",
        min_overlap=0.018,
        name="trigger pivot is mounted in handle boss",
    )
    ctx.expect_within(
        floor_head,
        lower_wand,
        axes="yz",
        inner_elem="nozzle_pin",
        outer_elem="nozzle_trunnion",
        margin=0.002,
        name="nozzle pin lies inside trunnion barrel envelope",
    )
    ctx.expect_within(
        lower_wand,
        upper_body,
        axes="yz",
        inner_elem="fold_pin",
        outer_elem="fold_barrel",
        margin=0.002,
        name="fold pin lies inside hinge barrel envelope",
    )

    rest_wand_aabb = ctx.part_element_world_aabb(lower_wand, elem="lower_tube")
    with ctx.pose({nozzle: 0.45}):
        pitched_wand_aabb = ctx.part_element_world_aabb(lower_wand, elem="lower_tube")
    ctx.check(
        "nozzle pitch tips the wand fore-aft",
        rest_wand_aabb is not None
        and pitched_wand_aabb is not None
        and abs(
            (pitched_wand_aabb[0][1] + pitched_wand_aabb[1][1])
            - (rest_wand_aabb[0][1] + rest_wand_aabb[1][1])
        )
        > 0.030,
        details=f"rest={rest_wand_aabb}, pitched={pitched_wand_aabb}",
    )

    rest_body_aabb = ctx.part_element_world_aabb(upper_body, elem="motor_housing")
    with ctx.pose({fold: 1.65}):
        folded_body_aabb = ctx.part_element_world_aabb(upper_body, elem="motor_housing")
    ctx.check(
        "fold hinge swings powered body downward",
        rest_body_aabb is not None
        and folded_body_aabb is not None
        and (folded_body_aabb[0][2] + folded_body_aabb[1][2])
        < (rest_body_aabb[0][2] + rest_body_aabb[1][2]) - 0.15,
        details=f"rest={rest_body_aabb}, folded={folded_body_aabb}",
    )

    ctx.check(
        "primary joints have realistic bounded travel",
        nozzle.motion_limits is not None
        and fold.motion_limits is not None
        and trigger_joint.motion_limits is not None
        and nozzle.motion_limits.lower < 0.0 < nozzle.motion_limits.upper
        and 1.5 < fold.motion_limits.upper < 2.4
        and 0.20 < trigger_joint.motion_limits.upper < 0.45,
        details="expected pitch, fold, and trigger travel limits",
    )

    return ctx.report()


object_model = build_object_model()
