from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PEDESTAL_LENGTH = 0.22
PEDESTAL_WIDTH = 0.15
PEDESTAL_BASE_HEIGHT = 0.04
PEDESTAL_BASE_CENTER_Z = -0.05

LINK1_LEN = 0.18
LINK2_LEN = 0.34
LINK3_LEN = 0.16

SHOULDER_HUB_RADIUS = 0.028
ELBOW_HUB_RADIUS = 0.022
WRIST_HUB_RADIUS = 0.018

LINK1_HUB_T = 0.012
LINK2_HUB_T = 0.010
LINK3_HUB_T = 0.008

PEDESTAL_CHEEK_T = 0.007
LINK1_CHEEK_T = 0.007
LINK2_CHEEK_T = 0.006

LINK1_BEAM_H = 0.03
LINK2_BEAM_H = 0.026
LINK3_BEAM_H = 0.018

HALF_PI = 1.5707963267948966


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_y_cylinder(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(HALF_PI, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_arm")

    dark_gray = model.material("dark_gray", rgba=(0.22, 0.24, 0.26, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    anodized_mid = model.material("anodized_mid", rgba=(0.52, 0.56, 0.60, 1.0))
    tool_finish = model.material("tool_finish", rgba=(0.82, 0.84, 0.87, 1.0))

    shoulder_y = LINK1_HUB_T / 2.0 + PEDESTAL_CHEEK_T / 2.0
    elbow_y = LINK2_HUB_T / 2.0 + LINK1_CHEEK_T / 2.0
    wrist_y = LINK3_HUB_T / 2.0 + LINK2_CHEEK_T / 2.0

    pedestal = model.part("pedestal")
    _add_box(
        pedestal,
        "pedestal_base",
        (PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_BASE_HEIGHT),
        (0.0, 0.0, PEDESTAL_BASE_CENTER_Z),
        dark_gray,
    )
    _add_box(
        pedestal,
        "pedestal_buttress_pos",
        (0.050, 0.020, 0.034),
        (-0.030, 0.020, -0.033),
        dark_gray,
    )
    _add_box(
        pedestal,
        "pedestal_buttress_neg",
        (0.050, 0.020, 0.034),
        (-0.030, -0.020, -0.033),
        dark_gray,
    )
    _add_box(
        pedestal,
        "pedestal_cheek_pos",
        (0.040, PEDESTAL_CHEEK_T, 0.060),
        (-0.004, shoulder_y, 0.0),
        dark_gray,
    )
    _add_box(
        pedestal,
        "pedestal_cheek_neg",
        (0.040, PEDESTAL_CHEEK_T, 0.060),
        (-0.004, -shoulder_y, 0.0),
        dark_gray,
    )

    link1 = model.part("link1")
    _add_y_cylinder(link1, "link1_hub", SHOULDER_HUB_RADIUS, LINK1_HUB_T, (0.0, 0.0, 0.0), satin_aluminum)
    _add_box(
        link1,
        "link1_beam",
        (0.118, LINK1_HUB_T, LINK1_BEAM_H),
        (0.069, 0.0, 0.0),
        satin_aluminum,
    )
    _add_box(
        link1,
        "link1_spine",
        (0.074, 0.018, 0.018),
        (0.083, 0.0, 0.0),
        satin_aluminum,
    )
    _add_box(
        link1,
        "link1_elbow_cheek_pos",
        (0.046, LINK1_CHEEK_T, 0.028),
        (LINK1_LEN - 0.023, elbow_y, 0.0),
        satin_aluminum,
    )
    _add_box(
        link1,
        "link1_elbow_web_pos",
        (0.018, LINK1_CHEEK_T, 0.020),
        (0.137, elbow_y, 0.0),
        satin_aluminum,
    )
    _add_box(
        link1,
        "link1_elbow_cheek_neg",
        (0.046, LINK1_CHEEK_T, 0.028),
        (LINK1_LEN - 0.023, -elbow_y, 0.0),
        satin_aluminum,
    )
    _add_box(
        link1,
        "link1_elbow_web_neg",
        (0.018, LINK1_CHEEK_T, 0.020),
        (0.137, -elbow_y, 0.0),
        satin_aluminum,
    )

    link2 = model.part("link2")
    _add_y_cylinder(link2, "link2_hub", ELBOW_HUB_RADIUS, LINK2_HUB_T, (0.0, 0.0, 0.0), anodized_mid)
    _add_box(
        link2,
        "link2_beam",
        (0.300, LINK2_HUB_T, LINK2_BEAM_H),
        (0.170, 0.0, 0.0),
        anodized_mid,
    )
    _add_box(
        link2,
        "link2_spine",
        (0.210, 0.016, 0.016),
        (0.155, 0.0, 0.0),
        anodized_mid,
    )
    _add_box(
        link2,
        "link2_wrist_cheek_pos",
        (0.044, LINK2_CHEEK_T, 0.022),
        (LINK2_LEN - 0.022, wrist_y, 0.0),
        anodized_mid,
    )
    _add_box(
        link2,
        "link2_wrist_web_pos",
        (0.032, LINK2_CHEEK_T, 0.018),
        (0.288, wrist_y, 0.0),
        anodized_mid,
    )
    _add_box(
        link2,
        "link2_wrist_cheek_neg",
        (0.044, LINK2_CHEEK_T, 0.022),
        (LINK2_LEN - 0.022, -wrist_y, 0.0),
        anodized_mid,
    )
    _add_box(
        link2,
        "link2_wrist_web_neg",
        (0.032, LINK2_CHEEK_T, 0.018),
        (0.288, -wrist_y, 0.0),
        anodized_mid,
    )

    link3 = model.part("link3")
    _add_y_cylinder(link3, "link3_hub", WRIST_HUB_RADIUS, LINK3_HUB_T, (0.0, 0.0, 0.0), satin_aluminum)
    _add_box(
        link3,
        "link3_beam",
        (0.132, LINK3_HUB_T, LINK3_BEAM_H),
        (0.070, 0.0, 0.0),
        satin_aluminum,
    )
    _add_box(
        link3,
        "link3_mount_pad",
        (0.024, LINK3_HUB_T, 0.026),
        (LINK3_LEN - 0.012, 0.0, 0.0),
        satin_aluminum,
    )

    tool_plate = model.part("tool_plate")
    _add_box(tool_plate, "tool_flange", (0.006, 0.006, 0.016), (0.003, 0.0, 0.0), tool_finish)
    _add_box(tool_plate, "tool_neck", (0.014, 0.006, 0.018), (0.012, 0.0, 0.0), tool_finish)
    _add_box(tool_plate, "tool_plate_body", (0.070, 0.006, 0.060), (0.053, 0.0, 0.0), tool_finish)
    _add_y_cylinder(tool_plate, "tool_center_boss", 0.008, 0.006, (0.018, 0.0, 0.0), tool_finish)

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=link1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-1.10, upper=1.25),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(LINK1_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.5, lower=-2.15, upper=0.20),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(LINK2_LEN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.0, lower=-1.70, upper=1.70),
    )
    model.articulation(
        "tool_mount",
        ArticulationType.FIXED,
        parent=link3,
        child=tool_plate,
        origin=Origin(xyz=(LINK3_LEN, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")
    tool_plate = object_model.get_part("tool_plate")

    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    wrist = object_model.get_articulation("wrist")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.expect_contact(link1, pedestal, name="shoulder_contact")
    ctx.expect_contact(link2, link1, name="elbow_contact")
    ctx.expect_contact(link3, link2, name="wrist_contact")
    ctx.expect_contact(tool_plate, link3, name="tool_mount_contact")

    ctx.check(
        "serial_joint_axes_are_coplanar",
        shoulder.axis == (0.0, 1.0, 0.0)
        and elbow.axis == (0.0, 1.0, 0.0)
        and wrist.axis == (0.0, 1.0, 0.0),
        details=f"axes were shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist.axis}",
    )

    link1_aabb = ctx.part_world_aabb(link1)
    link2_aabb = ctx.part_world_aabb(link2)
    link3_aabb = ctx.part_world_aabb(link3)
    pedestal_aabb = ctx.part_world_aabb(pedestal)
    tool_aabb = ctx.part_world_aabb(tool_plate)

    if all(aabb is not None for aabb in (link1_aabb, link2_aabb, link3_aabb, pedestal_aabb, tool_aabb)):
        link1_dx = link1_aabb[1][0] - link1_aabb[0][0]
        link2_dx = link2_aabb[1][0] - link2_aabb[0][0]
        link3_dy = link3_aabb[1][1] - link3_aabb[0][1]
        link3_dz = link3_aabb[1][2] - link3_aabb[0][2]
        pedestal_dz = pedestal_aabb[1][2] - pedestal_aabb[0][2]
        pedestal_dx = pedestal_aabb[1][0] - pedestal_aabb[0][0]
        tool_dx = tool_aabb[1][0] - tool_aabb[0][0]
        tool_dy = tool_aabb[1][1] - tool_aabb[0][1]
        tool_dz = tool_aabb[1][2] - tool_aabb[0][2]

        ctx.check(
            "link_proportions_read_as_serial_arm",
            link2_dx > link1_dx and link3_dy < link3_dz and pedestal_dz < pedestal_dx and tool_dy < tool_dz,
            details=(
                f"link1_dx={link1_dx:.4f}, link2_dx={link2_dx:.4f}, link3_dy={link3_dy:.4f}, "
                f"link3_dz={link3_dz:.4f}, pedestal_dz={pedestal_dz:.4f}, pedestal_dx={pedestal_dx:.4f}, "
                f"tool_dx={tool_dx:.4f}, tool_dy={tool_dy:.4f}, tool_dz={tool_dz:.4f}"
            ),
        )

    with ctx.pose({"shoulder": -0.55, "elbow": -1.10, "wrist": 0.45}):
        ctx.expect_contact(link1, pedestal, name="shoulder_contact_in_work_pose")
        ctx.expect_contact(link2, link1, name="elbow_contact_in_work_pose")
        ctx.expect_contact(link3, link2, name="wrist_contact_in_work_pose")
        ctx.expect_contact(tool_plate, link3, name="tool_mount_contact_in_work_pose")

        tool_pos = ctx.part_world_position(tool_plate)
        link3_pos = ctx.part_world_position(link3)
        link2_pos = ctx.part_world_position(link2)
        if tool_pos is not None and link3_pos is not None and link2_pos is not None:
            ctx.check(
                "serial_chain_lifts_monotonically_in_work_pose",
                tool_pos[2] > link3_pos[2] > link2_pos[2],
                details=f"tool_z={tool_pos[2]:.4f}, link3_z={link3_pos[2]:.4f}, link2_z={link2_pos[2]:.4f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
