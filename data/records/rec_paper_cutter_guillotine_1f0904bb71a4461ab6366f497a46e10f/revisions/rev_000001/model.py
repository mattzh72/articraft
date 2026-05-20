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
    model = ArticulatedObject(name="craft_paper_cutter")

    board_gray = model.material("board_gray", rgba=(0.76, 0.78, 0.80, 1.0))
    mat_white = model.material("mat_white", rgba=(0.92, 0.93, 0.91, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.11, 0.12, 1.0))
    amber = model.material("amber", rgba=(0.84, 0.60, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.50, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=board_gray,
        name="board_body",
    )
    base.visual(
        Box((0.30, 0.46, 0.0016)),
        origin=Origin(xyz=(0.012, 0.0, 0.0108)),
        material=mat_white,
        name="work_surface",
    )
    base.visual(
        Box((0.032, 0.50, 0.008)),
        origin=Origin(xyz=(-0.164, 0.0, 0.014)),
        material=dark_graphite,
        name="hinge_rail",
    )
    base.visual(
        Box((0.010, 0.44, 0.0015)),
        origin=Origin(xyz=(0.126, 0.0, 0.01075)),
        material=amber,
        name="cut_strip",
    )
    base.visual(
        Box((0.18, 0.022, 0.003)),
        origin=Origin(xyz=(0.035, -0.236, 0.0115)),
        material=brushed_steel,
        name="front_ruler",
    )
    base.visual(
        Box((0.008, 0.028, 0.010)),
        origin=Origin(xyz=(0.161, -0.235, 0.015)),
        material=dark_graphite,
        name="stop_hinge_block",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.50, 0.040)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Box((0.288, 0.49, 0.012)),
        origin=Origin(xyz=(0.144, 0.0, 0.0065)),
        material=brushed_steel,
        name="arm_panel",
    )
    blade_arm.visual(
        Box((0.024, 0.49, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, 0.009)),
        material=dark_graphite,
        name="hinge_stiffener",
    )
    blade_arm.visual(
        Box((0.006, 0.49, 0.003)),
        origin=Origin(xyz=(0.003, 0.0, 0.0)),
        material=dark_graphite,
        name="hinge_lip",
    )
    blade_arm.visual(
        Box((0.020, 0.49, 0.016)),
        origin=Origin(xyz=(0.278, 0.0, 0.008)),
        material=dark_graphite,
        name="blade_housing",
    )
    blade_arm.visual(
        Box((0.014, 0.44, 0.008)),
        origin=Origin(xyz=(0.256, 0.0, 0.004)),
        material=handle_black,
        name="pressure_strip",
    )
    handle_geom = tube_from_spline_points(
        [
            (0.232, -0.180, 0.012),
            (0.250, -0.176, 0.034),
            (0.258, -0.128, 0.048),
            (0.258, -0.055, 0.050),
            (0.250, -0.006, 0.034),
            (0.232, -0.002, 0.012),
        ],
        radius=0.006,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    blade_arm.visual(
        mesh_from_geometry(handle_geom, "cutter_handle_loop"),
        material=handle_black,
        name="handle_loop",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.288, 0.49, 0.060)),
        mass=0.85,
        origin=Origin(xyz=(0.144, 0.0, 0.030)),
    )

    stop_arm = model.part("stop_arm")
    stop_arm.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_graphite,
        name="stop_hinge_barrel",
    )
    stop_arm.visual(
        Box((0.012, 0.160, 0.004)),
        origin=Origin(xyz=(0.006, 0.080, 0.002)),
        material=brushed_steel,
        name="stop_bar",
    )
    stop_arm.visual(
        Box((0.012, 0.014, 0.009)),
        origin=Origin(xyz=(0.006, 0.153, 0.0045)),
        material=amber,
        name="stop_tab",
    )
    stop_arm.inertial = Inertial.from_geometry(
        Box((0.020, 0.170, 0.020)),
        mass=0.06,
        origin=Origin(xyz=(0.006, 0.085, 0.010)),
    )

    model.articulation(
        "blade_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(-0.152, 0.0, 0.0195)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "stop_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=stop_arm,
        origin=Origin(xyz=(0.173, -0.235, 0.0100)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=math.pi / 2.0,
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

    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    stop_arm = object_model.get_part("stop_arm")
    blade_hinge = object_model.get_articulation("blade_hinge")
    stop_hinge = object_model.get_articulation("stop_hinge")

    with ctx.pose({blade_hinge: 0.0, stop_hinge: 0.0}):
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            positive_elem="arm_panel",
            negative_elem="work_surface",
            min_gap=0.006,
            max_gap=0.015,
            name="blade arm rests just above the base board",
        )
        ctx.expect_overlap(
            blade_arm,
            base,
            axes="xy",
            min_overlap=0.20,
            name="closed blade arm covers the working area footprint",
        )
        ctx.expect_gap(
            stop_arm,
            base,
            axis="z",
            positive_elem="stop_bar",
            negative_elem="board_body",
            min_gap=0.0,
            max_gap=0.010,
            name="folded stop arm stays low over the base corner",
        )

    closed_handle_box = ctx.part_element_world_aabb(blade_arm, elem="handle_loop")
    with ctx.pose({blade_hinge: math.radians(60.0)}):
        open_handle_box = ctx.part_element_world_aabb(blade_arm, elem="handle_loop")
    ctx.check(
        "blade hinge opens the arm upward",
        closed_handle_box is not None
        and open_handle_box is not None
        and open_handle_box[0][2] > closed_handle_box[0][2] + 0.10,
        details=f"closed_handle_box={closed_handle_box}, open_handle_box={open_handle_box}",
    )

    with ctx.pose({stop_hinge: 0.0}):
        folded_stop_box = ctx.part_element_world_aabb(stop_arm, elem="stop_bar")
    with ctx.pose({stop_hinge: math.pi / 2.0}):
        deployed_stop_box = ctx.part_element_world_aabb(stop_arm, elem="stop_bar")

    folded_dx = None
    folded_dy = None
    deployed_dx = None
    deployed_dy = None
    if folded_stop_box is not None:
        folded_dx = folded_stop_box[1][0] - folded_stop_box[0][0]
        folded_dy = folded_stop_box[1][1] - folded_stop_box[0][1]
    if deployed_stop_box is not None:
        deployed_dx = deployed_stop_box[1][0] - deployed_stop_box[0][0]
        deployed_dy = deployed_stop_box[1][1] - deployed_stop_box[0][1]

    ctx.check(
        "stop arm folds from the side edge to the front edge",
        folded_dx is not None
        and folded_dy is not None
        and deployed_dx is not None
        and deployed_dy is not None
        and folded_dy > folded_dx * 4.0
        and deployed_dx > deployed_dy * 4.0,
        details=(
            f"folded_dx={folded_dx}, folded_dy={folded_dy}, "
            f"deployed_dx={deployed_dx}, deployed_dy={deployed_dy}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
