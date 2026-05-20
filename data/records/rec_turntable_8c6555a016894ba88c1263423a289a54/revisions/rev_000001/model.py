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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    plinth_black = model.material("plinth_black", rgba=(0.14, 0.14, 0.15, 1.0))
    top_plate = model.material("top_plate", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.77, 0.79, 1.0))
    center_label = model.material("center_label", rgba=(0.84, 0.84, 0.82, 1.0))
    tonearm_black = model.material("tonearm_black", rgba=(0.10, 0.10, 0.11, 1.0))
    tonearm_silver = model.material("tonearm_silver", rgba=(0.72, 0.74, 0.76, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.45, 0.35, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=plinth_black,
        name="plinth_body",
    )
    plinth.visual(
        Box((0.43, 0.33, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=top_plate,
        name="top_deck",
    )
    for index, (x, y) in enumerate(
        (
            (0.172, 0.122),
            (-0.172, 0.122),
            (0.172, -0.122),
            (-0.172, -0.122),
        )
    ):
        plinth.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"foot_{index}",
        )
    plinth.visual(
        Cylinder(radius=0.028, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.063)),
        material=aluminum,
        name="platter_bearing_cap",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.45, 0.35, 0.064)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.156, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=aluminum,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=rubber,
        name="slip_mat",
    )
    platter.visual(
        Cylinder(radius=0.045, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.02975)),
        material=center_label,
        name="center_label",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0335)),
        material=aluminum,
        name="spindle_tip",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.156, length=0.026),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=12.0),
    )

    tonearm_base_module = model.part("tonearm_base_module")
    tonearm_base_module.visual(
        Cylinder(radius=0.028, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=tonearm_black,
        name="mounting_disk",
    )
    tonearm_base_module.visual(
        Cylinder(radius=0.021, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=tonearm_black,
        name="support_pedestal",
    )
    tonearm_base_module.visual(
        Cylinder(radius=0.0035, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=tonearm_silver,
        name="pivot_pin",
    )
    tonearm_base_module.visual(
        Box((0.015, 0.018, 0.012)),
        origin=Origin(xyz=(-0.016, 0.0, 0.014)),
        material=tonearm_black,
        name="anti_skate_block",
    )
    tonearm_base_module.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(xyz=(0.024, -0.020, 0.012)),
        material=tonearm_silver,
        name="armrest_post",
    )
    tonearm_base_module.visual(
        Cylinder(radius=0.0035, length=0.018),
        origin=Origin(
            xyz=(0.020, -0.020, 0.024),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=tonearm_silver,
        name="armrest_cradle",
    )
    tonearm_base_module.inertial = Inertial.from_geometry(
        Box((0.070, 0.060, 0.028)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    arm_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.010),
                (0.002, -0.045, 0.011),
                (0.005, -0.135, 0.010),
                (0.008, -0.205, 0.008),
            ],
            radius=0.0048,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        "tonearm_tube",
    )

    tonearm_bearing_module = model.part("tonearm_bearing_module")
    tonearm_bearing_module.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=tonearm_black,
        name="gimbal_collar",
    )
    tonearm_bearing_module.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=tonearm_silver,
        name="pivot_turret",
    )
    tonearm_bearing_module.visual(
        arm_tube_mesh,
        material=tonearm_silver,
        name="arm_tube",
    )
    tonearm_bearing_module.visual(
        Cylinder(radius=0.004, length=0.030),
        origin=Origin(
            xyz=(0.0, 0.018, 0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=tonearm_silver,
        name="counterweight_stub",
    )
    tonearm_bearing_module.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(
            xyz=(0.0, 0.046, 0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=tonearm_black,
        name="counterweight",
    )
    tonearm_bearing_module.inertial = Inertial.from_geometry(
        Box((0.035, 0.270, 0.032)),
        mass=0.22,
        origin=Origin(xyz=(0.004, -0.070, 0.011)),
    )

    tonearm_head_module = model.part("tonearm_head_module")
    tonearm_head_module.visual(
        Cylinder(radius=0.0048, length=0.012),
        origin=Origin(
            xyz=(0.0, -0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=tonearm_silver,
        name="head_connector",
    )
    tonearm_head_module.visual(
        Box((0.018, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, -0.018, -0.003)),
        material=tonearm_silver,
        name="headshell_plate",
    )
    tonearm_head_module.visual(
        Box((0.014, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, -0.034, -0.003)),
        material=tonearm_silver,
        name="headshell_nose",
    )
    tonearm_head_module.visual(
        Box((0.014, 0.010, 0.007)),
        origin=Origin(xyz=(0.0, -0.033, -0.0085)),
        material=tonearm_black,
        name="cartridge_body",
    )
    tonearm_head_module.visual(
        Cylinder(radius=0.0015, length=0.018),
        origin=Origin(
            xyz=(0.009, -0.030, 0.0005),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=tonearm_silver,
        name="finger_lift",
    )
    tonearm_head_module.inertial = Inertial.from_geometry(
        Box((0.020, 0.040, 0.016)),
        mass=0.05,
        origin=Origin(xyz=(0.0, -0.020, -0.005)),
    )

    model.articulation(
        "plinth_to_tonearm_base",
        ArticulationType.FIXED,
        parent=plinth,
        child=tonearm_base_module,
        origin=Origin(xyz=(0.168, 0.096, 0.062)),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=tonearm_base_module,
        child=tonearm_bearing_module,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.2,
            lower=0.0,
            upper=0.85,
        ),
    )
    model.articulation(
        "bearing_to_head",
        ArticulationType.FIXED,
        parent=tonearm_bearing_module,
        child=tonearm_head_module,
        origin=Origin(xyz=(0.008, -0.205, 0.008)),
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
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm_base_module = object_model.get_part("tonearm_base_module")
    tonearm_bearing_module = object_model.get_part("tonearm_bearing_module")
    tonearm_head_module = object_model.get_part("tonearm_head_module")
    tonearm_swing = object_model.get_articulation("tonearm_swing")

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        min_gap=0.001,
        max_gap=0.004,
        positive_elem="platter_disc",
        negative_elem="top_deck",
        name="platter clears the plinth deck",
    )
    ctx.expect_contact(
        platter,
        plinth,
        elem_a="platter_disc",
        elem_b="platter_bearing_cap",
        name="platter is supported on the center bearing cap",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        min_overlap=0.28,
        name="platter stays centered on the plinth footprint",
    )
    ctx.expect_contact(
        tonearm_base_module,
        plinth,
        elem_a="mounting_disk",
        elem_b="top_deck",
        name="tonearm base module mounts onto the plinth deck",
    )
    ctx.expect_gap(
        tonearm_bearing_module,
        tonearm_base_module,
        axis="z",
        min_gap=0.0015,
        max_gap=0.0035,
        positive_elem="gimbal_collar",
        negative_elem="support_pedestal",
        name="bearing module is visibly separated above the base module",
    )
    ctx.expect_origin_gap(
        tonearm_bearing_module,
        tonearm_head_module,
        axis="y",
        min_gap=0.18,
        name="head module projects forward from the bearing module",
    )

    rest_head_position = ctx.part_world_position(tonearm_head_module)
    with ctx.pose({tonearm_swing: 0.55}):
        ctx.expect_overlap(
            tonearm_head_module,
            platter,
            axes="xy",
            min_overlap=0.010,
            name="swung tonearm head reaches over the platter",
        )
        swung_head_position = ctx.part_world_position(tonearm_head_module)

    ctx.check(
        "positive tonearm swing moves the head inward over the record",
        rest_head_position is not None
        and swung_head_position is not None
        and swung_head_position[0] < rest_head_position[0] - 0.06,
        details=f"rest={rest_head_position}, swung={swung_head_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
