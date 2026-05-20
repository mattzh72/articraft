from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_vacuum_cleaner")

    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.12, 0.13, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    copper = model.material("copper", rgba=(0.74, 0.45, 0.19, 1.0))
    clear_bin = model.material("clear_bin", rgba=(0.72, 0.80, 0.86, 0.35))

    main_body = model.part("main_body")
    main_body.inertial = Inertial.from_geometry(
        Box((0.18, 0.22, 0.34)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.01, 0.12)),
    )

    main_body.visual(
        Cylinder(radius=0.044, length=0.14),
        origin=Origin(xyz=(0.0, 0.045, 0.135), rpy=(pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="motor_housing",
    )
    main_body.visual(
        Cylinder(radius=0.036, length=0.17),
        origin=Origin(xyz=(0.0, 0.082, 0.115), rpy=(pi / 2.0, 0.0, 0.0)),
        material=clear_bin,
        name="dust_bin",
    )
    main_body.visual(
        Box((0.088, 0.030, 0.090)),
        origin=Origin(xyz=(0.0, -0.030, 0.045)),
        material=charcoal,
        name="yoke_bridge",
    )
    main_body.visual(
        Box((0.012, 0.036, 0.130)),
        origin=Origin(xyz=(0.038, 0.000, 0.028)),
        material=charcoal,
        name="left_cheek",
    )
    main_body.visual(
        Box((0.012, 0.036, 0.130)),
        origin=Origin(xyz=(-0.038, 0.000, 0.028)),
        material=charcoal,
        name="right_cheek",
    )
    main_body.visual(
        Box((0.090, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, 0.000, 0.085)),
        material=dark_gray,
        name="upper_crossmember",
    )
    main_body.visual(
        Box((0.075, 0.070, 0.105)),
        origin=Origin(xyz=(0.0, -0.050, 0.145), rpy=(0.18, 0.0, 0.0)),
        material=charcoal,
        name="battery_pack",
    )
    main_body.visual(
        Cylinder(radius=0.020, length=0.12),
        origin=Origin(xyz=(0.0, -0.018, 0.265), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="handle_grip",
    )
    handle_geom = tube_from_spline_points(
        [
            (0.0, -0.030, 0.080),
            (0.0, -0.058, 0.150),
            (0.0, -0.046, 0.225),
            (0.0, -0.010, 0.265),
        ],
        radius=0.014,
        samples_per_segment=16,
        radial_segments=18,
    )
    main_body.visual(
        mesh_from_geometry(handle_geom, "vacuum_handle_loop"),
        material=charcoal,
        name="handle_loop",
    )
    main_body.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.0, 0.120, 0.120), rpy=(pi / 2.0, 0.0, 0.0)),
        material=copper,
        name="cyclone_shroud",
    )

    upper_wand = model.part("upper_wand")
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.056, 0.032, 0.450)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
    )
    upper_wand.visual(
        Cylinder(radius=0.013, length=0.056),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="shoulder_barrel",
    )
    upper_wand.visual(
        Cylinder(radius=0.016, length=0.410),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=steel,
        name="upper_tube",
    )
    upper_wand.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=aluminum,
        name="shoulder_collar",
    )
    upper_wand.visual(
        Cylinder(radius=0.018, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.360)),
        material=aluminum,
        name="upper_cuff",
    )

    lower_wand = model.part("lower_wand")
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.052, 0.030, 0.390)),
        mass=0.48,
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
    )
    lower_wand.visual(
        Cylinder(radius=0.0115, length=0.052),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="elbow_barrel",
    )
    lower_wand.visual(
        Cylinder(radius=0.0145, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=steel,
        name="lower_tube",
    )
    lower_wand.visual(
        Cylinder(radius=0.0175, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.321)),
        material=aluminum,
        name="lower_cuff",
    )
    lower_wand.visual(
        Box((0.040, 0.024, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=aluminum,
        name="elbow_socket",
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.290, 0.095, 0.060)),
        mass=0.62,
        origin=Origin(xyz=(0.0, 0.030, -0.026)),
    )
    floor_nozzle.visual(
        Box((0.290, 0.086, 0.030)),
        origin=Origin(xyz=(0.0, 0.038, -0.015)),
        material=charcoal,
        name="nozzle_housing",
    )
    floor_nozzle.visual(
        Box((0.050, 0.032, 0.060)),
        origin=Origin(xyz=(0.0, -0.006, -0.030)),
        material=dark_gray,
        name="neck_socket",
    )
    floor_nozzle.visual(
        Box((0.200, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.000, -0.024)),
        material=dark_gray,
        name="rear_axle_bar",
    )
    floor_nozzle.visual(
        Box((0.260, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.079, -0.024)),
        material=copper,
        name="front_lip",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.014, length=0.015),
        origin=Origin(xyz=(0.108, 0.006, -0.029), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="left_roller",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.014, length=0.015),
        origin=Origin(xyz=(-0.108, 0.006, -0.029), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_gray,
        name="right_roller",
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=upper_wand,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.85,
        ),
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=(0.0, 0.0, -0.420)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.2,
            lower=-0.15,
            upper=1.10,
        ),
    )
    model.articulation(
        "lower_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.0, 0.0, -0.350)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.45,
        ),
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

    main_body = object_model.get_part("main_body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    shoulder = object_model.get_articulation("body_to_upper_wand")
    elbow = object_model.get_articulation("upper_to_lower_wand")
    nozzle_pitch = object_model.get_articulation("lower_to_floor_nozzle")

    ctx.expect_gap(
        main_body,
        upper_wand,
        axis="z",
        min_gap=0.0,
        max_gap=0.05,
        positive_elem="yoke_bridge",
        negative_elem="shoulder_collar",
        name="shoulder collar sits just below body yoke",
    )
    ctx.expect_within(
        upper_wand,
        main_body,
        axes="x",
        inner_elem="shoulder_barrel",
        margin=0.0,
        name="shoulder barrel stays framed between the body cheeks",
    )
    rest_cuff = ctx.part_element_world_aabb(upper_wand, elem="upper_cuff")
    with ctx.pose({shoulder: 0.65}):
        folded_cuff = ctx.part_element_world_aabb(upper_wand, elem="upper_cuff")
        ctx.check(
            "upper wand can fold forward",
            rest_cuff is not None
            and folded_cuff is not None
            and folded_cuff[0][1] + folded_cuff[1][1] > rest_cuff[0][1] + rest_cuff[1][1] + 0.18,
            details=f"rest={rest_cuff}, folded={folded_cuff}",
        )
    ctx.expect_gap(
        upper_wand,
        lower_wand,
        axis="z",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="upper_cuff",
        negative_elem="elbow_socket",
        name="lower wand elbow sits directly below the upper cuff",
    )
    ctx.expect_gap(
        lower_wand,
        floor_nozzle,
        axis="z",
        min_gap=0.0,
        max_gap=0.020,
        positive_elem="lower_cuff",
        negative_elem="neck_socket",
        name="nozzle neck sits directly under the lower wand cuff",
    )
    rest_lower = ctx.part_element_world_aabb(lower_wand, elem="lower_cuff")
    with ctx.pose({shoulder: 0.45, elbow: 0.80}):
        folded_lower = ctx.part_element_world_aabb(lower_wand, elem="lower_cuff")
        ctx.check(
            "wand chain bends forward at the elbows",
            rest_lower is not None
            and folded_lower is not None
            and folded_lower[0][1] + folded_lower[1][1] > rest_lower[0][1] + rest_lower[1][1] + 0.30,
            details=f"rest={rest_lower}, folded={folded_lower}",
        )
    rest_front = ctx.part_element_world_aabb(floor_nozzle, elem="front_lip")
    with ctx.pose({nozzle_pitch: -0.40}):
        dipped_front = ctx.part_element_world_aabb(floor_nozzle, elem="front_lip")
        ctx.check(
            "floor nozzle pitches nose-down",
            rest_front is not None
            and dipped_front is not None
            and dipped_front[0][2] < rest_front[0][2] - 0.015,
            details=f"rest={rest_front}, dipped={dipped_front}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
