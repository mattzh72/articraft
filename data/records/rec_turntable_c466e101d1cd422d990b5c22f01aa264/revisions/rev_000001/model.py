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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    plinth_black = model.material("plinth_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.07, 0.07, 0.08, 1.0))
    matte_black = model.material("matte_black", rgba=(0.04, 0.04, 0.05, 1.0))
    aluminum = model.material("aluminum", rgba=(0.75, 0.77, 0.80, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.66, 0.68, 0.71, 1.0))
    label_yellow = model.material("label_yellow", rgba=(0.83, 0.72, 0.30, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.44, 0.35, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=plinth_black,
        name="plinth_body",
    )
    plinth.visual(
        Box((0.40, 0.31, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=brushed_aluminum,
        name="top_plate",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (0.165, 0.125),
            (0.165, -0.125),
            (-0.165, 0.125),
            (-0.165, -0.125),
        )
    ):
        plinth.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(x_pos, y_pos, -0.006)),
            material=satin_black,
            name=f"foot_{index}",
        )
    plinth.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.155, -0.090, 0.056)),
        material=brushed_aluminum,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.020, length=0.005),
        origin=Origin(xyz=(-0.045, 0.0, 0.050)),
        material=satin_black,
        name="spindle_bearing",
    )
    plinth.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.155, -0.090, 0.071)),
        material=satin_black,
        name="tonearm_cap",
    )
    plinth.visual(
        Box((0.030, 0.016, 0.010)),
        origin=Origin(xyz=(0.175, 0.135, 0.051)),
        material=satin_black,
        name="power_switch",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.44, 0.35, 0.070)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.152, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.02575)),
        material=matte_black,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.055, length=0.001),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=label_yellow,
        name="label_disc",
    )
    platter.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=aluminum,
        name="spindle",
    )
    platter.visual(
        Box((0.012, 0.004, 0.0015)),
        origin=Origin(xyz=(0.141, 0.0, 0.02475)),
        material=matte_black,
        name="strobe_marker",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.152, length=0.032),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brushed_aluminum,
        name="pivot_collar",
    )
    tonearm.visual(
        Box((0.028, 0.020, 0.016)),
        origin=Origin(xyz=(0.006, 0.0, 0.016)),
        material=satin_black,
        name="gimbal_block",
    )
    tonearm.visual(
        Cylinder(radius=0.004, length=0.225),
        origin=Origin(xyz=(0.1125, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.030, 0.016, 0.004)),
        origin=Origin(xyz=(0.235, 0.0, 0.018)),
        material=satin_black,
        name="headshell",
    )
    tonearm.visual(
        Box((0.012, 0.012, 0.006)),
        origin=Origin(xyz=(0.244, 0.0, 0.015)),
        material=matte_black,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0035, length=0.050),
        origin=Origin(xyz=(-0.025, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(-0.045, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="counterweight",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.31, 0.040, 0.040)),
        mass=0.25,
        origin=Origin(xyz=(0.095, 0.0, 0.018)),
    )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(-0.045, 0.0, 0.0525)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(0.155, -0.090, 0.076), rpy=(0.0, 0.0, 2.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=0.0,
            upper=0.75,
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

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_joint = object_model.get_articulation("plinth_to_platter")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="top_plate",
        min_gap=0.0015,
        max_gap=0.0045,
        name="platter rides slightly above the deck",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_body",
        elem_b="plinth_body",
        min_overlap=0.30,
        name="platter remains large relative to the plinth footprint",
    )
    ctx.expect_gap(
        tonearm,
        plinth,
        axis="z",
        positive_elem="pivot_collar",
        negative_elem="tonearm_cap",
        max_gap=0.0005,
        max_penetration=0.0,
        name="tonearm collar seats on the tonearm base",
    )
    ctx.check(
        "platter uses continuous vertical rotation",
        platter_joint.articulation_type == ArticulationType.CONTINUOUS
        and platter_joint.axis == (0.0, 0.0, 1.0)
        and platter_joint.motion_limits is not None
        and platter_joint.motion_limits.lower is None
        and platter_joint.motion_limits.upper is None,
        details=str(
            {
                "type": platter_joint.articulation_type,
                "axis": platter_joint.axis,
                "limits": platter_joint.motion_limits,
            }
        ),
    )
    ctx.check(
        "tonearm uses bounded vertical-axis swing",
        tonearm_joint.articulation_type == ArticulationType.REVOLUTE
        and tonearm_joint.axis == (0.0, 0.0, 1.0)
        and tonearm_joint.motion_limits is not None
        and tonearm_joint.motion_limits.lower == 0.0
        and tonearm_joint.motion_limits.upper is not None
        and tonearm_joint.motion_limits.upper >= 0.6,
        details=str(
            {
                "type": tonearm_joint.articulation_type,
                "axis": tonearm_joint.axis,
                "limits": tonearm_joint.motion_limits,
            }
        ),
    )

    platter_center = aabb_center(ctx.part_world_aabb(platter))
    rest_marker = aabb_center(ctx.part_element_world_aabb(platter, elem="strobe_marker"))
    rest_headshell = aabb_center(ctx.part_element_world_aabb(tonearm, elem="headshell"))

    with ctx.pose({"plinth_to_platter": 1.2, "plinth_to_tonearm": 0.55}):
        spun_marker = aabb_center(ctx.part_element_world_aabb(platter, elem="strobe_marker"))
        swung_headshell = aabb_center(ctx.part_element_world_aabb(tonearm, elem="headshell"))

    marker_rest_radius = None
    marker_spun_radius = None
    headshell_rest_dist = None
    headshell_swung_dist = None
    if platter_center is not None and rest_marker is not None and spun_marker is not None:
        marker_rest_radius = math.hypot(
            rest_marker[0] - platter_center[0],
            rest_marker[1] - platter_center[1],
        )
        marker_spun_radius = math.hypot(
            spun_marker[0] - platter_center[0],
            spun_marker[1] - platter_center[1],
        )
    if platter_center is not None and rest_headshell is not None and swung_headshell is not None:
        headshell_rest_dist = math.hypot(
            rest_headshell[0] - platter_center[0],
            rest_headshell[1] - platter_center[1],
        )
        headshell_swung_dist = math.hypot(
            swung_headshell[0] - platter_center[0],
            swung_headshell[1] - platter_center[1],
        )

    ctx.check(
        "platter marker circles around the spindle when spun",
        platter_center is not None
        and rest_marker is not None
        and spun_marker is not None
        and math.hypot(rest_marker[0] - spun_marker[0], rest_marker[1] - spun_marker[1]) > 0.10
        and marker_rest_radius is not None
        and marker_spun_radius is not None
        and abs(marker_rest_radius - marker_spun_radius) < 0.005,
        details=str(
            {
                "platter_center": platter_center,
                "rest_marker": rest_marker,
                "spun_marker": spun_marker,
                "rest_radius": marker_rest_radius,
                "spun_radius": marker_spun_radius,
            }
        ),
    )
    ctx.check(
        "tonearm swings inward toward the platter",
        rest_headshell is not None
        and swung_headshell is not None
        and headshell_rest_dist is not None
        and headshell_swung_dist is not None
        and swung_headshell[0] < rest_headshell[0] - 0.05
        and headshell_swung_dist < headshell_rest_dist - 0.05,
        details=str(
            {
                "rest_headshell": rest_headshell,
                "swung_headshell": swung_headshell,
                "rest_dist_to_platter": headshell_rest_dist,
                "swung_dist_to_platter": headshell_swung_dist,
            }
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
