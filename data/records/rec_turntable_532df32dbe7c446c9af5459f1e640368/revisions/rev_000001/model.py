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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = model.material("walnut", rgba=(0.42, 0.28, 0.17, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    matte_black = model.material("matte_black", rgba=(0.06, 0.07, 0.08, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))

    plinth = model.part("plinth")

    plinth_shell = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.48, 0.38, 0.020, corner_segments=10),
            0.068,
        ),
        "plinth_shell",
    )
    plinth.visual(plinth_shell, material=walnut, name="plinth_shell")
    plinth.visual(
        Box((0.452, 0.352, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=satin_black,
        name="top_plate",
    )
    plinth.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(-0.175, -0.126, 0.071)),
        material=aluminum,
        name="speed_knob",
    )
    plinth.visual(
        Box((0.078, 0.070, 0.004)),
        origin=Origin(xyz=(0.158, 0.088, 0.066)),
        material=satin_black,
        name="armboard",
    )
    plinth.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.160, 0.092, 0.083)),
        material=matte_black,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Box((0.010, 0.014, 0.024)),
        origin=Origin(xyz=(0.208, -0.020, 0.080)),
        material=aluminum,
        name="arm_rest_post",
    )
    plinth.visual(
        Box((0.018, 0.010, 0.004)),
        origin=Origin(xyz=(0.208, -0.020, 0.094)),
        material=aluminum,
        name="arm_rest_cradle",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.48, 0.38, 0.074)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.136, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.128, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=matte_black,
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.040, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0236)),
        material=satin_black,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.002, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=aluminum,
        name="spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.136, length=0.023),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=matte_black,
        name="pivot_cap",
    )
    tonearm.visual(
        Box((0.028, 0.024, 0.012)),
        origin=Origin(xyz=(0.012, 0.0, 0.010)),
        material=matte_black,
        name="gimbal_block",
    )
    tonearm.visual(
        Cylinder(radius=0.0048, length=0.194),
        origin=Origin(xyz=(0.109, 0.0, 0.013), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.020, 0.018, 0.004)),
        origin=Origin(xyz=(0.214, 0.0, 0.008)),
        material=aluminum,
        name="headshell",
    )
    tonearm.visual(
        Box((0.012, 0.009, 0.008)),
        origin=Origin(xyz=(0.224, 0.0, 0.003)),
        material=matte_black,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0009, length=0.004),
        origin=Origin(xyz=(0.226, 0.0, -0.001)),
        material=aluminum,
        name="stylus_tip",
    )
    tonearm.visual(
        Cylinder(radius=0.0038, length=0.040),
        origin=Origin(xyz=(-0.020, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(-0.049, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="counterweight",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.290, 0.040, 0.028)),
        mass=0.35,
        origin=Origin(xyz=(0.095, 0.0, 0.011)),
    )

    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.160, 0.092, 0.098), rpy=(0.0, 0.0, -1.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=1.5,
            lower=-1.30,
            upper=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm")

    plinth_aabb = ctx.part_world_aabb(plinth)
    platter_aabb = ctx.part_world_aabb(platter)
    plinth_x_span = None if plinth_aabb is None else plinth_aabb[1][0] - plinth_aabb[0][0]
    plinth_y_span = None if plinth_aabb is None else plinth_aabb[1][1] - plinth_aabb[0][1]
    platter_x_span = None if platter_aabb is None else platter_aabb[1][0] - platter_aabb[0][0]
    platter_y_span = None if platter_aabb is None else platter_aabb[1][1] - platter_aabb[0][1]
    ctx.check(
        "platter stays visually smaller than the plinth",
        (
            plinth_x_span is not None
            and plinth_y_span is not None
            and platter_x_span is not None
            and platter_y_span is not None
            and platter_x_span <= 0.62 * plinth_x_span
            and platter_y_span <= 0.74 * plinth_y_span
        ),
        details=(
            f"plinth_spans=({plinth_x_span}, {plinth_y_span}), "
            f"platter_spans=({platter_x_span}, {platter_y_span})"
        ),
    )

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="top_plate",
        max_gap=0.0002,
        max_penetration=0.0,
        name="platter sits directly on the top plate",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="pivot_cap",
        negative_elem="tonearm_pedestal",
        max_gap=0.0002,
        max_penetration=0.0,
        name="tonearm pivot cap seats on pedestal",
    )

    rest_tip = ctx.part_element_world_aabb(tonearm, elem="stylus_tip")
    with ctx.pose({tonearm_joint: -1.05}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="stylus_tip",
            elem_b="rubber_mat",
            min_overlap=0.001,
            name="tonearm can sweep over the platter area",
        )
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="stylus_tip",
            negative_elem="rubber_mat",
            min_gap=0.001,
            max_gap=0.006,
            name="stylus hovers just above the record surface",
        )
        play_tip = ctx.part_element_world_aabb(tonearm, elem="stylus_tip")

    rest_center_x = None if rest_tip is None else 0.5 * (rest_tip[0][0] + rest_tip[1][0])
    play_center_x = None if play_tip is None else 0.5 * (play_tip[0][0] + play_tip[1][0])
    ctx.check(
        "tonearm swings inward toward the platter",
        rest_center_x is not None and play_center_x is not None and play_center_x < rest_center_x - 0.10,
        details=f"rest_center_x={rest_center_x}, play_center_x={play_center_x}",
    )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
