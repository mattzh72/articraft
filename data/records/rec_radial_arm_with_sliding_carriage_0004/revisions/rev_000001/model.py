from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shop_radial_arm_module", assets=ASSETS)

    machine_gray = model.material("machine_gray", rgba=(0.64, 0.67, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.28, 0.31, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.90, 0.47, 0.14, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.22, 0.16, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=dark_steel,
        name="foot_base",
    )
    pedestal.visual(
        Cylinder(radius=0.035, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=machine_gray,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.050, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=dark_steel,
        name="shoulder_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.042, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=safety_orange,
        name="column_band",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.22, 0.16, 0.15)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    beam = model.part("beam")
    beam.visual(
        Cylinder(radius=0.049, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=machine_gray,
        name="shoulder_table",
    )
    beam.visual(
        Box((0.34, 0.07, 0.035)),
        origin=Origin(xyz=(0.19, 0.0, 0.0355)),
        material=machine_gray,
        name="beam_body",
    )
    beam.visual(
        Box((0.30, 0.012, 0.012)),
        origin=Origin(xyz=(0.19, 0.022, 0.059)),
        material=dark_steel,
        name="rail_left",
    )
    beam.visual(
        Box((0.30, 0.012, 0.012)),
        origin=Origin(xyz=(0.19, -0.022, 0.059)),
        material=dark_steel,
        name="rail_right",
    )
    beam.visual(
        Box((0.014, 0.07, 0.045)),
        origin=Origin(xyz=(0.353, 0.0, 0.0405)),
        material=dark_steel,
        name="end_stop",
    )
    beam.inertial = Inertial.from_geometry(
        Box((0.36, 0.07, 0.07)),
        mass=8.5,
        origin=Origin(xyz=(0.18, 0.0, 0.035)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.085, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.022, 0.005)),
        material=dark_steel,
        name="shoe_left",
    )
    carriage.visual(
        Box((0.085, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.022, 0.005)),
        material=dark_steel,
        name="shoe_right",
    )
    carriage.visual(
        Box((0.085, 0.092, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=safety_orange,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.055, 0.050, 0.034)),
        origin=Origin(xyz=(0.022, 0.0, 0.055)),
        material=machine_gray,
        name="tool_block",
    )
    carriage.visual(
        Box((0.022, 0.028, 0.022)),
        origin=Origin(xyz=(0.050, 0.0, 0.028)),
        material=dark_steel,
        name="tool_nose",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.10, 0.10, 0.08)),
        mass=3.2,
        origin=Origin(xyz=(0.01, 0.0, 0.03)),
    )

    model.articulation(
        "shoulder_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(0.11, 0.0, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.30,
            lower=0.0,
            upper=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    pedestal = object_model.get_part("pedestal")
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    shoulder_yaw = object_model.get_articulation("shoulder_yaw")
    carriage_slide = object_model.get_articulation("carriage_slide")

    shoulder_plinth = pedestal.get_visual("shoulder_plinth")
    shoulder_table = beam.get_visual("shoulder_table")
    rail_left = beam.get_visual("rail_left")
    rail_right = beam.get_visual("rail_right")
    shoe_left = carriage.get_visual("shoe_left")
    shoe_right = carriage.get_visual("shoe_right")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.expect_contact(
        beam,
        pedestal,
        contact_tol=5e-4,
        elem_a=shoulder_table,
        elem_b=shoulder_plinth,
        name="beam shoulder table seats on pedestal plinth",
    )
    ctx.expect_overlap(
        beam,
        pedestal,
        axes="xy",
        elem_a=shoulder_table,
        elem_b=shoulder_plinth,
        min_overlap=0.095,
        name="beam shoulder overlaps pedestal plinth footprint",
    )
    ctx.expect_gap(
        carriage,
        beam,
        axis="z",
        positive_elem=shoe_left,
        negative_elem=rail_left,
        max_gap=5e-4,
        max_penetration=0.0,
        name="left carriage shoe rides on left rail",
    )
    ctx.expect_gap(
        carriage,
        beam,
        axis="z",
        positive_elem=shoe_right,
        negative_elem=rail_right,
        max_gap=5e-4,
        max_penetration=0.0,
        name="right carriage shoe rides on right rail",
    )
    ctx.expect_within(
        carriage,
        beam,
        axes="x",
        inner_elem=shoe_left,
        outer_elem=rail_left,
        margin=0.0,
        name="left shoe starts within rail travel span",
    )

    with ctx.pose({carriage_slide: 0.15}):
        ctx.expect_within(
            carriage,
            beam,
            axes="x",
            inner_elem=shoe_left,
            outer_elem=rail_left,
            margin=0.0,
            name="left shoe stays within rail travel span at full extension",
        )

    with ctx.pose({carriage_slide: 0.0}):
        rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.15}):
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage has about 150 mm of travel",
        rest_pos is not None
        and extended_pos is not None
        and abs((extended_pos[0] - rest_pos[0]) - 0.15) <= 0.003
        and abs(extended_pos[1] - rest_pos[1]) <= 1e-6
        and abs(extended_pos[2] - rest_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({shoulder_yaw: math.pi / 2.0, carriage_slide: 0.0}):
        swung_pos = ctx.part_world_position(carriage)
    ctx.check(
        "shoulder yaws carriage around vertical axis",
        rest_pos is not None
        and swung_pos is not None
        and abs(swung_pos[0]) <= 0.01
        and abs(swung_pos[1] - 0.11) <= 0.01
        and abs(swung_pos[2] - rest_pos[2]) <= 1e-6,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    with ctx.pose({shoulder_yaw: math.pi / 2.0, carriage_slide: 0.15}):
        swung_extended_pos = ctx.part_world_position(carriage)
    ctx.check(
        "prismatic carriage follows beam after shoulder rotation",
        swung_pos is not None
        and swung_extended_pos is not None
        and abs(swung_extended_pos[0] - swung_pos[0]) <= 0.01
        and abs((swung_extended_pos[1] - swung_pos[1]) - 0.15) <= 0.01,
        details=f"swung={swung_pos}, swung_extended={swung_extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
