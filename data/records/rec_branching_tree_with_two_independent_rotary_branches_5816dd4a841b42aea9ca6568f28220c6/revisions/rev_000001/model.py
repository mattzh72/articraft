from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PI = math.pi


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_arm_positioning_tree")

    dark = _mat(model, "dark_powder_coat", (0.08, 0.085, 0.09, 1.0))
    mast_paint = _mat(model, "graphite_mast", (0.18, 0.20, 0.21, 1.0))
    blue = _mat(model, "blue_bearing_housings", (0.05, 0.18, 0.32, 1.0))
    steel = _mat(model, "brushed_steel", (0.58, 0.60, 0.58, 1.0))
    bolt = _mat(model, "black_oxide_fasteners", (0.015, 0.015, 0.014, 1.0))
    yellow = _mat(model, "safety_yellow_arms", (0.94, 0.66, 0.10, 1.0))
    orange = _mat(model, "orange_service_arm", (0.88, 0.32, 0.06, 1.0))
    rubber = _mat(model, "rubber_tool_faces", (0.025, 0.026, 0.025, 1.0))
    seam = _mat(model, "shadow_split_lines", (0.0, 0.0, 0.0, 1.0))

    mast = model.part("mast")

    # Grounded pedestal and central column.
    mast.visual(Box((0.50, 0.38, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.0225)), material=dark, name="base_plate")
    mast.visual(Cylinder(radius=0.058, length=1.58), origin=Origin(xyz=(0.0, 0.0, 0.79)), material=mast_paint, name="central_tube")
    mast.visual(Cylinder(radius=0.090, length=0.070), origin=Origin(xyz=(0.0, 0.0, 0.075)), material=steel, name="base_collar")
    for i, (x, y) in enumerate(((-0.19, -0.13), (-0.19, 0.13), (0.19, -0.13), (0.19, 0.13))):
        mast.visual(
            Cylinder(radius=0.017, length=0.025),
            origin=Origin(xyz=(x, y, 0.047)),
            material=bolt,
            name=f"base_bolt_{i}",
        )

    # Upper hub package: forward split bearing carrier, visibly different from the lower side hub.
    upper_z = 1.43
    upper_y = 0.18
    mast.visual(Box((0.24, 0.085, 0.115)), origin=Origin(xyz=(0.0, -0.010, upper_z)), material=blue, name="upper_mast_clamp")
    mast.visual(Box((0.19, 0.18, 0.120)), origin=Origin(xyz=(0.0, 0.045, upper_z - 0.165)), material=blue, name="upper_saddle")
    mast.visual(Box((0.23, 0.18, 0.080)), origin=Origin(xyz=(0.0, upper_y, upper_z - 0.095)), material=blue, name="upper_lower_carrier")
    mast.visual(Box((0.23, 0.18, 0.080)), origin=Origin(xyz=(0.0, upper_y, upper_z + 0.095)), material=blue, name="upper_cap_carrier")
    mast.visual(Box((0.022, 0.185, 0.010)), origin=Origin(xyz=(0.102, upper_y, upper_z + 0.001)), material=seam, name="upper_split_shadow")
    for sx in (-1.0, 1.0):
        mast.visual(
            Box((0.030, 0.185, 0.235)),
            origin=Origin(xyz=(sx * 0.116, upper_y, upper_z)),
            material=blue,
            name=f"upper_side_strap_{0 if sx < 0 else 1}",
        )
    mast.visual(
        Cylinder(radius=0.066, length=0.180),
        origin=Origin(xyz=(0.0, upper_y, upper_z), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=steel,
        name="upper_bushing",
    )
    for i, x in enumerate((-0.070, 0.070)):
        mast.visual(
            Cylinder(radius=0.012, length=0.020),
            origin=Origin(xyz=(x, upper_y - 0.045, upper_z + 0.142), rpy=(-PI / 2.0, 0.0, 0.0)),
            material=bolt,
            name=f"upper_cap_bolt_{i}",
        )
        mast.visual(
            Cylinder(radius=0.012, length=0.020),
            origin=Origin(xyz=(x, upper_y + 0.045, upper_z - 0.142), rpy=(-PI / 2.0, 0.0, 0.0)),
            material=bolt,
            name=f"upper_lower_bolt_{i}",
        )
    mast.visual(Box((0.10, 0.030, 0.030)), origin=Origin(xyz=(-0.090, upper_y + 0.080, upper_z + 0.090)), material=bolt, name="upper_stop_0")
    mast.visual(Box((0.10, 0.030, 0.030)), origin=Origin(xyz=(0.090, upper_y + 0.080, upper_z - 0.090)), material=bolt, name="upper_stop_1")

    # Lower hub package: side-mounted bolted flange and gusseted bearing drum.
    lower_z = 0.98
    lower_x = 0.17
    mast.visual(Box((0.095, 0.25, 0.125)), origin=Origin(xyz=(0.0, 0.0, lower_z)), material=blue, name="lower_mast_band")
    mast.visual(Box((0.19, 0.13, 0.075)), origin=Origin(xyz=(0.070, 0.0, lower_z - 0.115)), material=blue, name="lower_saddle_foot")
    mast.visual(Box((0.19, 0.045, 0.155)), origin=Origin(xyz=(0.075, -0.085, lower_z - 0.020), rpy=(0.0, 0.23, 0.0)), material=blue, name="lower_rear_gusset")
    mast.visual(Box((0.19, 0.045, 0.155)), origin=Origin(xyz=(0.075, 0.085, lower_z - 0.020), rpy=(0.0, 0.23, 0.0)), material=blue, name="lower_front_gusset")
    mast.visual(
        Cylinder(radius=0.088, length=0.200),
        origin=Origin(xyz=(lower_x, 0.0, lower_z), rpy=(0.0, PI / 2.0, 0.0)),
        material=steel,
        name="lower_bearing_drum",
    )
    mast.visual(Box((0.046, 0.245, 0.062)), origin=Origin(xyz=(lower_x, 0.0, lower_z + 0.114)), material=blue, name="lower_flange_top")
    mast.visual(Box((0.046, 0.245, 0.062)), origin=Origin(xyz=(lower_x, 0.0, lower_z - 0.114)), material=blue, name="lower_flange_bottom")
    mast.visual(Box((0.046, 0.062, 0.225)), origin=Origin(xyz=(lower_x, 0.114, lower_z)), material=blue, name="lower_flange_front")
    mast.visual(Box((0.046, 0.062, 0.225)), origin=Origin(xyz=(lower_x, -0.114, lower_z)), material=blue, name="lower_flange_rear")
    mast.visual(Box((0.080, 0.040, 0.050)), origin=Origin(xyz=(lower_x + 0.035, 0.132, lower_z + 0.055)), material=bolt, name="lower_stop_0")
    mast.visual(Box((0.080, 0.040, 0.050)), origin=Origin(xyz=(lower_x + 0.035, -0.132, lower_z - 0.055)), material=bolt, name="lower_stop_1")
    for i, (y, z) in enumerate(((-0.100, 0.100), (0.100, 0.100), (-0.100, -0.100), (0.100, -0.100))):
        mast.visual(
            Cylinder(radius=0.011, length=0.030),
            origin=Origin(xyz=(lower_x + 0.027, y, lower_z + z), rpy=(0.0, PI / 2.0, 0.0)),
            material=bolt,
            name=f"lower_flange_bolt_{i}",
        )

    forward_branch = model.part("forward_branch")
    forward_branch.visual(
        Cylinder(radius=0.044, length=0.240),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=steel,
        name="forward_trunnion",
    )
    forward_branch.visual(
        Cylinder(radius=0.070, length=0.045),
        origin=Origin(xyz=(0.0, 0.126, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=yellow,
        name="forward_drive_collar",
    )
    forward_branch.visual(Box((0.080, 0.050, 0.070)), origin=Origin(xyz=(0.0, 0.155, 0.0)), material=yellow, name="forward_neck")
    forward_branch.visual(Box((0.075, 0.420, 0.060)), origin=Origin(xyz=(0.0, 0.385, 0.0)), material=yellow, name="forward_beam")
    forward_branch.visual(Box((0.025, 0.310, 0.090)), origin=Origin(xyz=(-0.043, 0.400, -0.005)), material=yellow, name="forward_gusset_0")
    forward_branch.visual(Box((0.025, 0.310, 0.090)), origin=Origin(xyz=(0.043, 0.400, -0.005)), material=yellow, name="forward_gusset_1")
    forward_branch.visual(Box((0.150, 0.036, 0.125)), origin=Origin(xyz=(0.0, 0.612, 0.0)), material=yellow, name="forward_pad_back")
    forward_branch.visual(
        Cylinder(radius=0.078, length=0.038),
        origin=Origin(xyz=(0.0, 0.637, 0.0), rpy=(-PI / 2.0, 0.0, 0.0)),
        material=rubber,
        name="forward_tool_pad",
    )
    for i, (x, z) in enumerate(((-0.045, -0.045), (-0.045, 0.045), (0.045, -0.045), (0.045, 0.045))):
        forward_branch.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(xyz=(x, 0.655, z), rpy=(-PI / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"forward_pad_pin_{i}",
        )

    side_branch = model.part("side_branch")
    side_branch.visual(
        Cylinder(radius=0.045, length=0.240),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=steel,
        name="side_trunnion",
    )
    side_branch.visual(
        Cylinder(radius=0.072, length=0.045),
        origin=Origin(xyz=(0.126, 0.0, 0.0), rpy=(0.0, PI / 2.0, 0.0)),
        material=orange,
        name="side_drive_collar",
    )
    side_branch.visual(Box((0.086, 0.082, 0.070)), origin=Origin(xyz=(0.168, 0.0, 0.0)), material=orange, name="side_neck")
    side_branch.visual(Box((0.670, 0.070, 0.058)), origin=Origin(xyz=(0.530, 0.0, 0.0)), material=orange, name="side_beam")
    side_branch.visual(Box((0.470, 0.035, 0.090)), origin=Origin(xyz=(0.535, -0.038, -0.006)), material=orange, name="side_web_rear")
    side_branch.visual(Box((0.470, 0.035, 0.090)), origin=Origin(xyz=(0.535, 0.038, -0.006)), material=orange, name="side_web_front")
    side_branch.visual(Box((0.070, 0.078, 0.205)), origin=Origin(xyz=(0.895, 0.0, -0.090)), material=orange, name="side_drop_bracket")
    side_branch.visual(Box((0.035, 0.145, 0.105)), origin=Origin(xyz=(0.940, 0.0, -0.185)), material=orange, name="side_pad_back")
    side_branch.visual(
        Cylinder(radius=0.073, length=0.042),
        origin=Origin(xyz=(0.969, 0.0, -0.185), rpy=(0.0, PI / 2.0, 0.0)),
        material=rubber,
        name="side_tool_pad",
    )
    for i, (y, z) in enumerate(((-0.043, -0.218), (-0.043, -0.152), (0.043, -0.218), (0.043, -0.152))):
        side_branch.visual(
            Cylinder(radius=0.0075, length=0.010),
            origin=Origin(xyz=(0.988, y, z), rpy=(0.0, PI / 2.0, 0.0)),
            material=steel,
            name=f"side_pad_pin_{i}",
        )

    model.articulation(
        "mast_to_forward_branch",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=forward_branch,
        origin=Origin(xyz=(0.0, upper_y, upper_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.52, upper=0.52),
    )

    model.articulation(
        "mast_to_side_branch",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=side_branch,
        origin=Origin(xyz=(lower_x, 0.0, lower_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.9, lower=-0.62, upper=0.62),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    forward_branch = object_model.get_part("forward_branch")
    side_branch = object_model.get_part("side_branch")
    forward_joint = object_model.get_articulation("mast_to_forward_branch")
    side_joint = object_model.get_articulation("mast_to_side_branch")

    ctx.allow_overlap(
        mast,
        forward_branch,
        elem_a="upper_bushing",
        elem_b="forward_trunnion",
        reason="The forward trunnion is intentionally captured inside the split bearing bushing proxy.",
    )
    ctx.allow_overlap(
        mast,
        side_branch,
        elem_a="lower_bearing_drum",
        elem_b="side_trunnion",
        reason="The side trunnion is intentionally captured inside the bolted bearing drum proxy.",
    )

    ctx.expect_within(
        forward_branch,
        mast,
        axes="xz",
        inner_elem="forward_trunnion",
        outer_elem="upper_bushing",
        margin=0.002,
        name="forward trunnion is centered in upper bushing",
    )
    ctx.expect_overlap(
        forward_branch,
        mast,
        axes="y",
        elem_a="forward_trunnion",
        elem_b="upper_bushing",
        min_overlap=0.14,
        name="forward trunnion has retained bearing length",
    )
    ctx.expect_gap(
        forward_branch,
        mast,
        axis="y",
        positive_elem="forward_beam",
        negative_elem="upper_cap_carrier",
        min_gap=0.030,
        name="forward arm clears upper split carrier",
    )
    ctx.expect_within(
        side_branch,
        mast,
        axes="yz",
        inner_elem="side_trunnion",
        outer_elem="lower_bearing_drum",
        margin=0.002,
        name="side trunnion is centered in lower drum",
    )
    ctx.expect_overlap(
        side_branch,
        mast,
        axes="x",
        elem_a="side_trunnion",
        elem_b="lower_bearing_drum",
        min_overlap=0.16,
        name="side trunnion has retained bearing length",
    )
    ctx.expect_gap(
        side_branch,
        mast,
        axis="x",
        positive_elem="side_beam",
        negative_elem="lower_bearing_drum",
        min_gap=0.045,
        name="side arm clears lower bearing package",
    )

    # Decisive limit-pose checks: the articulated arms sweep around their support
    # axes without entering the fixed covers or each other.
    for q in (-0.52, 0.52):
        with ctx.pose({forward_joint: q}):
            ctx.expect_gap(
                forward_branch,
                mast,
                axis="y",
                positive_elem="forward_beam",
                negative_elem="upper_cap_carrier",
                min_gap=0.030,
                name=f"forward branch carrier clearance at {q:+.2f} rad",
            )
            ctx.expect_gap(
                forward_branch,
                side_branch,
                axis="z",
                positive_elem="forward_beam",
                negative_elem="side_beam",
                min_gap=0.24,
                name=f"upper forward branch stays above side branch at {q:+.2f} rad",
            )

    for q in (-0.62, 0.62):
        with ctx.pose({side_joint: q}):
            ctx.expect_gap(
                side_branch,
                mast,
                axis="x",
                positive_elem="side_beam",
                negative_elem="lower_bearing_drum",
                min_gap=0.045,
                name=f"side branch clears flange at {q:+.2f} rad",
            )

    return ctx.report()


object_model = build_object_model()
