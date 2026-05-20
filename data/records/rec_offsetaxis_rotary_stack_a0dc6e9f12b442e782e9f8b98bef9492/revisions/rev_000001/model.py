from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_W = 0.24
PLATE_H = 0.36
PLATE_T = 0.016
LOWER_AXIS_Z = -0.07
LOWER_PEDESTAL_R = 0.047
LOWER_PEDESTAL_LEN = 0.032
LOWER_FLANGE_R = 0.062
LOWER_FLANGE_LEN = 0.007

PLATFORM_R = 0.074
PLATFORM_T = 0.016
PLATFORM_HUB_R = 0.043
PLATFORM_HUB_T = 0.024

SECOND_AXIS_X = 0.205
SECOND_AXIS_Y = 0.1575
SECOND_AXIS_Z = 0.08
SECOND_HOUSING_R = 0.05
SECOND_HOUSING_LEN = 0.036
SECOND_COLLAR_R = 0.058
SECOND_COLLAR_LEN = 0.008

ROTOR_BARREL_R = 0.034
ROTOR_BARREL_LEN = 0.024
ROTOR_FLANGE_R = 0.044
ROTOR_FLANGE_LEN = 0.010
ROTOR_NOSE_R = 0.018
ROTOR_NOSE_LEN = 0.022


def make_backplate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_W, PLATE_T, PLATE_H)
        .edges("|Y")
        .fillet(0.012)
    )
    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.078, 0.11),
                (0.078, 0.11),
                (-0.078, -0.13),
                (0.078, -0.13),
            ]
        )
        .slot2D(0.016, 0.008, 90)
        .cutThruAll()
    )

    lower_pedestal = (
        cq.Workplane("XZ")
        .center(0.0, LOWER_AXIS_Z)
        .circle(LOWER_PEDESTAL_R)
        .extrude(LOWER_PEDESTAL_LEN)
        .translate((0.0, PLATE_T / 2.0 + LOWER_PEDESTAL_LEN, 0.0))
    )
    lower_flange = (
        cq.Workplane("XZ")
        .center(0.0, LOWER_AXIS_Z)
        .circle(LOWER_FLANGE_R)
        .extrude(LOWER_FLANGE_LEN)
        .translate((0.0, PLATE_T / 2.0 + LOWER_PEDESTAL_LEN, 0.0))
    )
    side_brace = (
        cq.Workplane("XY")
        .box(0.03, 0.026, 0.11)
        .translate((0.045, PLATE_T / 2.0 + 0.013, LOWER_AXIS_Z + 0.004))
    )
    lower_web = (
        cq.Workplane("XY")
        .box(0.104, 0.018, 0.034)
        .translate((0.0, PLATE_T / 2.0 + 0.009, LOWER_AXIS_Z - 0.042))
    )

    return (
        plate.union(lower_pedestal)
        .union(lower_flange)
        .union(side_brace)
        .union(side_brace.mirror("YZ"))
        .union(lower_web)
    )


def make_carrier_shape() -> cq.Workplane:
    lower_platform = (
        cq.Workplane("XZ").circle(PLATFORM_R).extrude(PLATFORM_T).translate((0.0, PLATFORM_T, 0.0))
    )
    lower_hub = (
        cq.Workplane("XZ").circle(PLATFORM_HUB_R).extrude(PLATFORM_HUB_T).translate((0.0, PLATFORM_HUB_T, 0.0))
    )
    arm_root = (
        cq.Workplane("XY")
        .box(0.12, 0.08, 0.045)
        .translate((0.045, 0.055, 0.015))
    )
    arm_beam = (
        cq.Workplane("XY")
        .box(0.12, 0.085, 0.034)
        .translate((0.115, 0.115, 0.03))
    )
    lower_rib = (
        cq.Workplane("XY")
        .box(0.17, 0.026, 0.028)
        .translate((0.11, 0.096, -0.004))
    )
    upright = (
        cq.Workplane("XY")
        .box(0.06, 0.05, 0.13)
        .translate((0.195, 0.122, 0.08))
    )
    second_housing = (
        cq.Workplane("XZ")
        .center(SECOND_AXIS_X, SECOND_AXIS_Z)
        .circle(SECOND_HOUSING_R)
        .extrude(SECOND_HOUSING_LEN)
        .translate((0.0, SECOND_AXIS_Y, 0.0))
    )
    second_collar = (
        cq.Workplane("XZ")
        .center(SECOND_AXIS_X, SECOND_AXIS_Z)
        .circle(SECOND_COLLAR_R)
        .extrude(SECOND_COLLAR_LEN)
        .translate((0.0, SECOND_AXIS_Y, 0.0))
    )

    return (
        lower_platform.union(lower_hub)
        .union(arm_root)
        .union(arm_beam)
        .union(lower_rib)
        .union(upright)
        .union(second_housing)
        .union(second_collar)
    )


def make_rotor_body_shape() -> cq.Workplane:
    barrel = (
        cq.Workplane("XZ")
        .circle(ROTOR_BARREL_R)
        .extrude(ROTOR_BARREL_LEN)
        .translate((0.0, ROTOR_BARREL_LEN, 0.0))
    )
    flange = (
        cq.Workplane("XZ")
        .circle(ROTOR_FLANGE_R)
        .extrude(ROTOR_FLANGE_LEN)
        .translate((0.0, ROTOR_BARREL_LEN + ROTOR_FLANGE_LEN, 0.0))
    )
    nose = (
        cq.Workplane("XZ")
        .circle(ROTOR_NOSE_R)
        .extrude(ROTOR_NOSE_LEN)
        .translate((0.0, ROTOR_BARREL_LEN + ROTOR_FLANGE_LEN + ROTOR_NOSE_LEN, 0.0))
    )
    return barrel.union(flange).union(nose)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_offset_rotary_bracket")

    backplate_mat = model.material("backplate_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    carrier_mat = model.material("carrier_finish", rgba=(0.45, 0.47, 0.50, 1.0))
    rotor_mat = model.material("rotor_finish", rgba=(0.73, 0.74, 0.77, 1.0))
    tab_mat = model.material("tab_finish", rgba=(0.16, 0.20, 0.24, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(make_backplate_shape(), "backplate"),
        material=backplate_mat,
        name="backplate_shell",
    )

    carrier = model.part("carrier")
    carrier.visual(
        mesh_from_cadquery(make_carrier_shape(), "carrier"),
        material=carrier_mat,
        name="carrier_frame",
    )

    cartridge = model.part("cartridge")
    cartridge.visual(
        mesh_from_cadquery(make_rotor_body_shape(), "cartridge_body"),
        material=rotor_mat,
        name="cartridge_body",
    )
    cartridge.visual(
        Box((0.10, 0.012, 0.024)),
        origin=Origin(xyz=(0.05, 0.038, 0.0)),
        material=tab_mat,
        name="drive_tab",
    )

    model.articulation(
        "backplate_to_carrier",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=carrier,
        origin=Origin(xyz=(0.0, PLATE_T / 2.0 + LOWER_PEDESTAL_LEN, LOWER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=-0.9,
            upper=1.25,
        ),
    )
    model.articulation(
        "carrier_to_cartridge",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=cartridge,
        origin=Origin(xyz=(SECOND_AXIS_X, SECOND_AXIS_Y, SECOND_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.0,
            lower=-2.2,
            upper=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    carrier = object_model.get_part("carrier")
    cartridge = object_model.get_part("cartridge")
    lower_joint = object_model.get_articulation("backplate_to_carrier")
    cartridge_joint = object_model.get_articulation("carrier_to_cartridge")

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

    ctx.expect_contact(
        carrier,
        backplate,
        name="lower_platform_contacts_backplate_pedestal",
    )
    ctx.expect_contact(
        cartridge,
        carrier,
        name="cartridge_contacts_carrier_housing",
    )
    ctx.expect_origin_gap(
        cartridge,
        backplate,
        axis="y",
        min_gap=0.16,
        name="offset_cartridge_projects_forward_of_backplate",
    )
    ctx.expect_origin_distance(
        cartridge,
        carrier,
        axes="xz",
        min_dist=0.19,
        max_dist=0.24,
        name="second_axis_is_offset_from_lower_axis",
    )

    axes_parallel = all(
        abs(a - b) < 1e-9 for a, b in zip(lower_joint.axis, cartridge_joint.axis)
    )
    ctx.check(
        "parallel_revolute_axes",
        axes_parallel,
        details=f"lower axis={lower_joint.axis}, cartridge axis={cartridge_joint.axis}",
    )

    with ctx.pose({lower_joint: 0.0, cartridge_joint: 0.0}):
        rest_cartridge_pos = ctx.part_world_position(cartridge)
        rest_tab_aabb = ctx.part_element_world_aabb(cartridge, elem="drive_tab")

    with ctx.pose({lower_joint: 0.8, cartridge_joint: 0.0}):
        swung_cartridge_pos = ctx.part_world_position(cartridge)

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))

    ctx.check(
        "lower_platform_positive_rotation_lifts_carried_axis",
        (
            rest_cartridge_pos is not None
            and swung_cartridge_pos is not None
            and swung_cartridge_pos[2] > rest_cartridge_pos[2] + 0.10
        ),
        details=(
            f"rest={rest_cartridge_pos}, swung={swung_cartridge_pos}, "
            "expected the offset axis to rise in +Z for positive lower-platform rotation"
        ),
    )

    with ctx.pose({lower_joint: 0.0, cartridge_joint: 1.0}):
        turned_tab_aabb = ctx.part_element_world_aabb(cartridge, elem="drive_tab")

    rest_tab_center = aabb_center(rest_tab_aabb)
    turned_tab_center = aabb_center(turned_tab_aabb)
    ctx.check(
        "cartridge_rotation_moves_drive_tab",
        (
            rest_tab_center is not None
            and turned_tab_center is not None
            and turned_tab_center[2] > rest_tab_center[2] + 0.03
        ),
        details=(
            f"rest_tab_center={rest_tab_center}, turned_tab_center={turned_tab_center}, "
            "expected the off-axis tab to swing upward in +Z under positive cartridge rotation"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
