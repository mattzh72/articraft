from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LoftSection,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


CARD_LENGTH = 0.170
CARD_HEIGHT = 0.111
CARD_THICKNESS = 0.043
FAN_CENTER_X = 0.086
FAN_CENTER_Y = 0.013
FAN_CENTER_Z = 0.066
FAN_OPENING_RADIUS = 0.046
ROTOR_RADIUS = 0.040


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 32,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * cos((2.0 * pi * i) / segments),
            cy + radius * sin((2.0 * pi * i) / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_itx_gpu")

    pcb_green = model.material("pcb_green", rgba=(0.10, 0.26, 0.16, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.18, 0.20, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.29, 0.31, 1.0))
    aluminum = model.material("aluminum", rgba=(0.66, 0.68, 0.71, 1.0))
    silver = model.material("silver", rgba=(0.78, 0.80, 0.82, 1.0))
    gold = model.material("gold", rgba=(0.84, 0.71, 0.34, 1.0))

    body = model.part("card_body")

    body.visual(
        Box((CARD_LENGTH, 0.0022, CARD_HEIGHT)),
        origin=Origin(xyz=(CARD_LENGTH * 0.5, -0.010, CARD_HEIGHT * 0.5)),
        material=pcb_green,
        name="pcb",
    )
    body.visual(
        Box((0.062, 0.0032, 0.006)),
        origin=Origin(xyz=(0.060, -0.010, 0.003)),
        material=gold,
        name="pcie_edge",
    )
    body.visual(
        Box((0.002, CARD_THICKNESS, 0.120)),
        origin=Origin(xyz=(-0.001, 0.0, 0.060)),
        material=silver,
        name="io_bracket",
    )
    body.visual(
        Box((0.010, 0.004, 0.107)),
        origin=Origin(xyz=(0.005, 0.0195, 0.0565)),
        material=silver,
        name="bracket_return_flange",
    )
    body.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.004, 0.019, 0.010)),
        material=gunmetal,
        name="foot_mount_block",
    )
    body.visual(
        Box((0.124, 0.021, 0.012)),
        origin=Origin(xyz=(0.094, -0.006, 0.033)),
        material=dark_gray,
        name="heatsink_base",
    )
    for fin_index in range(12):
        y = -0.0165 + fin_index * 0.00135
        body.visual(
            Box((0.118, 0.0011, 0.076)),
            origin=Origin(xyz=(0.097, y, 0.068)),
            material=aluminum,
            name=f"fin_{fin_index:02d}",
        )
    body.visual(
        Cylinder(radius=0.0042, length=0.006),
        origin=Origin(
            xyz=(FAN_CENTER_X, 0.004, FAN_CENTER_Z),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=gunmetal,
        name="fan_spindle",
    )
    body.visual(
        Box((0.094, 0.002, 0.004)),
        origin=Origin(xyz=(FAN_CENTER_X, 0.004, FAN_CENTER_Z)),
        material=gunmetal,
        name="fan_stator_bar_x",
    )
    body.visual(
        Box((0.004, 0.002, 0.094)),
        origin=Origin(xyz=(FAN_CENTER_X, 0.004, FAN_CENTER_Z)),
        material=gunmetal,
        name="fan_stator_bar_z",
    )

    shroud_bezel = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.145, 0.104, 0.012, corner_segments=10),
        [_circle_profile(FAN_OPENING_RADIUS, segments=40)],
        0.022,
        cap=True,
        center=True,
        closed=True,
    )
    shroud_bezel.rotate_x(pi * 0.5).translate(FAN_CENTER_X, 0.011, FAN_CENTER_Z)
    body.visual(
        mesh_from_geometry(shroud_bezel, "gpu_shroud_bezel"),
        material=matte_black,
        name="shroud_bezel",
    )
    front_nose = section_loft(
        SectionLoftSpec(
            sections=(
                LoftSection(
                    points=(
                        (0.136, -0.004, 0.022),
                        (0.136, 0.018, 0.022),
                        (0.136, 0.018, 0.061),
                        (0.136, -0.001, 0.054),
                    )
                ),
                LoftSection(
                    points=(
                        (0.153, -0.001, 0.024),
                        (0.153, 0.018, 0.024),
                        (0.153, 0.017, 0.058),
                        (0.153, 0.001, 0.052),
                    )
                ),
                LoftSection(
                    points=(
                        (0.169, 0.001, 0.026),
                        (0.169, 0.017, 0.026),
                        (0.169, 0.016, 0.054),
                        (0.169, 0.003, 0.049),
                    )
                ),
            ),
            cap=True,
            solid=True,
            repair="auto",
        )
    )
    body.visual(
        mesh_from_geometry(front_nose, "gpu_front_nose"),
        material=dark_gray,
        name="front_nose",
    )

    rotor = model.part("fan_rotor")
    rotor.visual(
        Cylinder(radius=0.0115, length=0.010),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=dark_gray,
        name="fan_hub",
    )
    rotor.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=silver,
        name="hub_cap",
    )
    blade_count = 7
    for blade_index in range(blade_count):
        angle = (2.0 * pi * blade_index) / blade_count
        rotor.visual(
            Box((0.030, 0.0025, 0.012)),
            origin=Origin(
                xyz=(0.0175 * cos(angle), 0.0, 0.0175 * sin(angle)),
                rpy=(0.0, angle + 0.32, 0.18),
            ),
            material=matte_black,
            name=f"blade_{blade_index + 1}",
        )
    rotor.visual(
        Cylinder(radius=ROTOR_RADIUS, length=0.002),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(pi * 0.5, 0.0, 0.0)),
        material=gunmetal,
        name="rotor_backplate",
    )

    foot = model.part("support_foot")
    foot.visual(
        Cylinder(radius=0.0030, length=0.006),
        origin=Origin(rpy=(pi * 0.5, 0.0, 0.0)),
        material=gunmetal,
        name="foot_hinge_barrel",
    )
    foot.visual(
        Box((0.044, 0.004, 0.006)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material=gunmetal,
        name="foot_leg",
    )
    foot.visual(
        Box((0.010, 0.005, 0.017)),
        origin=Origin(xyz=(0.047, 0.0, -0.0055)),
        material=dark_gray,
        name="foot_pad",
    )
    foot.visual(
        Box((0.016, 0.003, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, -0.004)),
        material=dark_gray,
        name="foot_rib",
    )

    model.articulation(
        "body_to_fan",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=rotor,
        origin=Origin(xyz=(FAN_CENTER_X, FAN_CENTER_Y, FAN_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=60.0),
    )
    model.articulation(
        "body_to_foot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=foot,
        origin=Origin(xyz=(0.004, 0.025, 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.10,
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
    body = object_model.get_part("card_body")
    rotor = object_model.get_part("fan_rotor")
    foot = object_model.get_part("support_foot")
    fan_joint = object_model.get_articulation("body_to_fan")
    foot_joint = object_model.get_articulation("body_to_foot")

    ctx.check(
        "fan is a continuous axial articulation",
        fan_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 4) for v in fan_joint.axis) == (0.0, 1.0, 0.0)
        and fan_joint.motion_limits is not None
        and fan_joint.motion_limits.lower is None
        and fan_joint.motion_limits.upper is None,
        details=f"type={fan_joint.articulation_type}, axis={fan_joint.axis}, limits={fan_joint.motion_limits}",
    )
    ctx.check(
        "support foot hinges from the bracket edge",
        foot_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 4) for v in foot_joint.axis) == (0.0, 1.0, 0.0)
        and foot_joint.motion_limits is not None
        and foot_joint.motion_limits.lower == 0.0
        and foot_joint.motion_limits.upper is not None
        and foot_joint.motion_limits.upper >= 1.0,
        details=f"type={foot_joint.articulation_type}, axis={foot_joint.axis}, limits={foot_joint.motion_limits}",
    )

    ctx.expect_within(
        rotor,
        body,
        axes="xz",
        margin=0.0,
        name="fan rotor stays within the shroud silhouette",
    )
    ctx.expect_overlap(
        rotor,
        body,
        axes="xz",
        min_overlap=0.075,
        name="fan is centered over the compact card face",
    )
    ctx.expect_contact(
        rotor,
        body,
        elem_a="rotor_backplate",
        elem_b="fan_spindle",
        contact_tol=0.001,
        name="fan rotor is supported by the central spindle",
    )
    ctx.expect_contact(
        foot,
        body,
        elem_a="foot_hinge_barrel",
        elem_b="foot_mount_block",
        contact_tol=0.001,
        name="support foot stays mounted to the bracket-side hinge block",
    )

    rest_aabb = ctx.part_world_aabb(foot)
    with ctx.pose({foot_joint: 1.0}):
        deployed_aabb = ctx.part_world_aabb(foot)
        deployed_origin = ctx.part_world_position(foot)
    rest_origin = ctx.part_world_position(foot)
    ctx.check(
        "support foot folds downward when deployed",
        rest_aabb is not None
        and deployed_aabb is not None
        and deployed_aabb[0][2] < rest_aabb[0][2] - 0.025,
        details=(
            f"rest_aabb={rest_aabb}, deployed_aabb={deployed_aabb}, "
            f"rest_origin={rest_origin}, deployed_origin={deployed_origin}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
