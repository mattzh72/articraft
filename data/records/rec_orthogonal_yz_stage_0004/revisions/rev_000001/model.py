from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yz_positioning_stage", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    anodized_tool = model.material("anodized_tool", rgba=(0.24, 0.43, 0.68, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((0.18, 0.39, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=painted_steel,
        name="base_plate",
    )
    base_frame.visual(
        Box((0.022, 0.30, 0.012)),
        origin=Origin(xyz=(-0.045, 0.0, 0.026)),
        material=rail_steel,
        name="left_y_rail",
    )
    base_frame.visual(
        Box((0.022, 0.30, 0.012)),
        origin=Origin(xyz=(0.045, 0.0, 0.026)),
        material=rail_steel,
        name="right_y_rail",
    )
    base_frame.visual(
        Box((0.16, 0.035, 0.03)),
        origin=Origin(xyz=(0.0, -0.1775, 0.035)),
        material=painted_steel,
        name="rear_end_block",
    )
    base_frame.visual(
        Box((0.16, 0.035, 0.03)),
        origin=Origin(xyz=(0.0, 0.1775, 0.035)),
        material=painted_steel,
        name="front_end_block",
    )
    base_frame.visual(
        Box((0.028, 0.28, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=rail_steel,
        name="y_screw_cover",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.18, 0.39, 0.05)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        Box((0.13, 0.07, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=aluminum,
        name="carriage_plate",
    )
    y_carriage.visual(
        Box((0.12, 0.018, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=aluminum,
        name="z_backplate",
    )
    y_carriage.visual(
        Box((0.014, 0.01, 0.18)),
        origin=Origin(xyz=(-0.034, 0.014, 0.145)),
        material=rail_steel,
        name="left_z_rail",
    )
    y_carriage.visual(
        Box((0.014, 0.01, 0.18)),
        origin=Origin(xyz=(0.034, 0.014, 0.145)),
        material=rail_steel,
        name="right_z_rail",
    )
    y_carriage.visual(
        Box((0.014, 0.01, 0.18)),
        origin=Origin(xyz=(0.0, 0.014, 0.145)),
        material=painted_steel,
        name="z_screw_cover",
    )
    y_carriage.visual(
        Box((0.09, 0.03, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.287)),
        material=painted_steel,
        name="z_top_cap",
    )
    y_carriage.inertial = Inertial.from_geometry(
        Box((0.13, 0.07, 0.30)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )

    z_carriage = model.part("z_carriage")
    z_carriage.visual(
        Box((0.088, 0.026, 0.055)),
        origin=Origin(xyz=(0.0, 0.013, 0.0275)),
        material=aluminum,
        name="z_carriage_body",
    )
    z_carriage.visual(
        Box((0.062, 0.012, 0.05)),
        origin=Origin(xyz=(0.0, 0.032, 0.018)),
        material=anodized_tool,
        name="tooling_plate",
    )
    z_carriage.inertial = Inertial.from_geometry(
        Box((0.09, 0.04, 0.07)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.02, 0.035)),
    )

    model.articulation(
        "base_to_y_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=-0.10,
            upper=0.10,
        ),
    )
    model.articulation(
        "y_carriage_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=y_carriage,
        child=z_carriage,
        origin=Origin(xyz=(0.0, 0.019, 0.065)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.25,
            lower=0.0,
            upper=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    y_carriage = object_model.get_part("y_carriage")
    z_carriage = object_model.get_part("z_carriage")

    y_slide = object_model.get_articulation("base_to_y_slide")
    z_slide = object_model.get_articulation("y_carriage_to_z_slide")

    base_left_y_rail = base_frame.get_visual("left_y_rail")
    carriage_plate = y_carriage.get_visual("carriage_plate")
    backplate = y_carriage.get_visual("z_backplate")
    left_z_rail = y_carriage.get_visual("left_z_rail")
    z_carriage_body = z_carriage.get_visual("z_carriage_body")
    tooling_plate = z_carriage.get_visual("tooling_plate")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=20,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        y_carriage,
        base_frame,
        axis="z",
        positive_elem=carriage_plate,
        negative_elem=base_left_y_rail,
        max_gap=0.001,
        max_penetration=0.0,
        name="y carriage seats on Y rail",
    )
    ctx.expect_overlap(
        y_carriage,
        base_frame,
        axes="xy",
        elem_a=carriage_plate,
        elem_b=base_left_y_rail,
        min_overlap=0.02,
        name="y carriage overlaps Y rail footprint",
    )
    ctx.expect_origin_gap(
        z_carriage,
        y_carriage,
        axis="z",
        min_gap=0.064,
        max_gap=0.066,
        name="z slide rest height",
    )
    ctx.expect_gap(
        z_carriage,
        y_carriage,
        axis="y",
        positive_elem=z_carriage_body,
        negative_elem=left_z_rail,
        max_gap=0.001,
        max_penetration=0.0,
        name="z carriage seats on vertical rail plane",
    )
    ctx.expect_overlap(
        z_carriage,
        y_carriage,
        axes="xz",
        elem_a=z_carriage_body,
        elem_b=backplate,
        min_overlap=0.05,
        name="z carriage remains backed by z slide structure",
    )
    ctx.expect_gap(
        z_carriage,
        z_carriage,
        axis="y",
        positive_elem=tooling_plate,
        negative_elem=z_carriage_body,
        max_gap=0.001,
        max_penetration=0.0,
        name="tooling plate mounts flush to z carriage",
    )

    with ctx.pose({y_slide: 0.10}):
        ctx.expect_origin_gap(
            y_carriage,
            base_frame,
            axis="y",
            min_gap=0.099,
            max_gap=0.101,
            name="y slide reaches positive travel",
        )
        ctx.expect_overlap(
            y_carriage,
            base_frame,
            axes="xy",
            elem_a=carriage_plate,
            elem_b=base_left_y_rail,
            min_overlap=0.02,
            name="y carriage stays guided at positive travel",
        )

    with ctx.pose({y_slide: -0.10}):
        ctx.expect_origin_gap(
            base_frame,
            y_carriage,
            axis="y",
            min_gap=0.099,
            max_gap=0.101,
            name="y slide reaches negative travel",
        )
        ctx.expect_overlap(
            y_carriage,
            base_frame,
            axes="xy",
            elem_a=carriage_plate,
            elem_b=base_left_y_rail,
            min_overlap=0.02,
            name="y carriage stays guided at negative travel",
        )

    with ctx.pose({z_slide: 0.15}):
        ctx.expect_origin_gap(
            z_carriage,
            y_carriage,
            axis="z",
            min_gap=0.214,
            max_gap=0.216,
            name="z slide reaches top travel",
        )
        ctx.expect_gap(
            z_carriage,
            y_carriage,
            axis="y",
            positive_elem=z_carriage_body,
            negative_elem=left_z_rail,
            max_gap=0.001,
            max_penetration=0.0,
            name="z carriage stays seated on rails at top travel",
        )

    with ctx.pose({y_slide: 0.10, z_slide: 0.15}):
        ctx.expect_gap(
            z_carriage,
            base_frame,
            axis="z",
            min_gap=0.16,
            name="tooling stays well above base at corner pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
