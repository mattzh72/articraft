from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_DEPTH = 0.028
ROOT_GAP = 0.016
ROOT_OUTER = 0.026
ROOT_SLOT_WIDTH = 0.050
ROOT_PIN_RADIUS = 0.0055

LINK1_LENGTH = 0.115
LINK2_LENGTH = 0.090


def _y_extruded_rect(center_x: float, center_z: float, size_x: float, size_z: float, depth: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .rect(size_x, size_z)
        .extrude(depth / 2.0, both=True)
    )


def _box(center_x: float, center_y: float, center_z: float, size_x: float, size_y: float, size_z: float) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate((center_x, center_y, center_z))


def _y_cylinder(center_x: float, center_z: float, radius: float, length_y: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(radius)
        .extrude(length_y / 2.0, both=True)
    )


def _add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_y_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _tapered_plate(top_width: float, bottom_width: float, top_z: float, bottom_z: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (top_width / 2.0, top_z),
                (bottom_width / 2.0, bottom_z),
                (-bottom_width / 2.0, bottom_z),
                (-top_width / 2.0, top_z),
            ]
        )
        .close()
        .extrude(thickness / 2.0, both=True)
    )


def _make_bridge_support() -> cq.Workplane:
    ear_thickness = (ROOT_OUTER - ROOT_GAP) / 2.0
    ear_center_y = ROOT_GAP / 2.0 + ear_thickness / 2.0

    shape = _box(0.0, 0.0, 0.110, 0.190, SUPPORT_DEPTH, 0.024)
    shape = shape.union(_box(-0.074, 0.0, 0.054, 0.022, SUPPORT_DEPTH, 0.088))
    shape = shape.union(_box(0.074, 0.0, 0.054, 0.022, SUPPORT_DEPTH, 0.088))
    shape = shape.union(_box(0.0, 0.0, 0.050, 0.024, 0.014, 0.100))
    shape = shape.union(_box(0.0, 0.0, 0.018, 0.034, 0.014, 0.032))
    shape = shape.union(_box(0.0, -ear_center_y, 0.001, 0.024, ear_thickness, 0.034))
    shape = shape.union(_box(0.0, ear_center_y, 0.001, 0.024, ear_thickness, 0.034))
    shape = shape.cut(_y_cylinder(0.0, 0.0, ROOT_PIN_RADIUS, ROOT_OUTER + 0.004))
    return shape


def _make_link_1() -> cq.Workplane:
    length = LINK1_LENGTH
    prox_eye_radius = 0.014
    prox_eye_len = 0.0154
    prox_hole_radius = ROOT_PIN_RADIUS
    distal_gap = 0.012
    distal_outer = 0.022
    distal_hole_radius = 0.0048
    ear_thickness = (distal_outer - distal_gap) / 2.0
    ear_center_y = distal_gap / 2.0 + ear_thickness / 2.0

    shape = _y_cylinder(0.0, 0.0, prox_eye_radius, prox_eye_len)
    shape = shape.union(_tapered_plate(0.030, 0.018, -0.012, -0.072, 0.010))
    shape = shape.union(_box(0.0, 0.0, -0.010, 0.024, 0.010, 0.010))
    shape = shape.union(_box(0.0, -ear_center_y, -0.091, 0.013, ear_thickness, 0.048))
    shape = shape.union(_box(0.0, ear_center_y, -0.091, 0.013, ear_thickness, 0.048))
    shape = shape.union(_box(0.0, -0.0085, -0.071, 0.016, 0.005, 0.010))
    shape = shape.union(_box(0.0, 0.0085, -0.071, 0.016, 0.005, 0.010))
    shape = shape.cut(_y_cylinder(0.0, 0.0, prox_hole_radius, prox_eye_len + 0.002))
    shape = shape.cut(_y_cylinder(0.0, -length, distal_hole_radius, distal_outer + 0.002))
    return shape


def _make_link_2() -> cq.Workplane:
    length = LINK2_LENGTH
    prox_eye_radius = 0.012
    prox_eye_len = 0.0114
    prox_hole_radius = 0.0048
    distal_gap = 0.010
    distal_outer = 0.018
    distal_hole_radius = 0.0042
    ear_thickness = (distal_outer - distal_gap) / 2.0
    ear_center_y = distal_gap / 2.0 + ear_thickness / 2.0

    shape = _y_cylinder(0.0, 0.0, prox_eye_radius, prox_eye_len)
    shape = shape.union(_tapered_plate(0.024, 0.015, -0.010, -0.056, 0.008))
    shape = shape.union(_box(0.0, 0.0, -0.008, 0.018, 0.008, 0.009))
    shape = shape.union(_box(0.0, -ear_center_y, -0.072, 0.011, ear_thickness, 0.036))
    shape = shape.union(_box(0.0, ear_center_y, -0.072, 0.011, ear_thickness, 0.036))
    shape = shape.union(_box(0.0, -0.007, -0.056, 0.013, 0.004, 0.008))
    shape = shape.union(_box(0.0, 0.007, -0.056, 0.013, 0.004, 0.008))
    shape = shape.cut(_y_cylinder(0.0, 0.0, prox_hole_radius, prox_eye_len + 0.002))
    shape = shape.cut(_y_cylinder(0.0, -length, distal_hole_radius, distal_outer + 0.002))
    return shape


def _make_end_link() -> cq.Workplane:
    prox_hub_radius = 0.010
    prox_hub_len = 0.0094
    prox_hole_radius = 0.0042
    body_width_top = 0.018
    body_width_bottom = 0.010
    body_thickness = 0.006
    body_length = 0.066
    tab_radius = 0.0065
    tab_center_z = -(body_length + 0.015)

    shape = _y_cylinder(0.0, 0.0, prox_hub_radius, prox_hub_len)
    shape = shape.union(_tapered_plate(body_width_top, body_width_bottom, -0.009, -0.070, body_thickness))
    shape = shape.union(_box(0.0, 0.0, -0.007, 0.015, body_thickness, 0.009))
    shape = shape.union(_box(0.0, 0.0, tab_center_z + 0.008, body_width_bottom, body_thickness, 0.016))
    shape = shape.union(_y_cylinder(0.0, tab_center_z, tab_radius, body_thickness))
    shape = shape.cut(_y_cylinder(0.0, 0.0, prox_hole_radius, prox_hub_len + 0.002))
    shape = shape.cut(_y_cylinder(0.0, tab_center_z, 0.0045, body_thickness + 0.010))
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_lever_chain")

    model.material("support_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("steel_large", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("steel_mid", rgba=(0.62, 0.65, 0.69, 1.0))
    model.material("steel_small", rgba=(0.55, 0.58, 0.62, 1.0))

    support = model.part("bridge_support")
    root_ear_t = (ROOT_OUTER - ROOT_GAP) / 2.0
    root_ear_y = ROOT_GAP / 2.0 + root_ear_t / 2.0
    _add_box(
        support,
        name="top_beam",
        size=(0.190, SUPPORT_DEPTH, 0.024),
        xyz=(0.0, 0.0, 0.110),
        material="support_gray",
    )
    _add_box(
        support,
        name="left_hanger",
        size=(0.022, SUPPORT_DEPTH, 0.086),
        xyz=(-0.074, 0.0, 0.067),
        material="support_gray",
    )
    _add_box(
        support,
        name="right_hanger",
        size=(0.022, SUPPORT_DEPTH, 0.086),
        xyz=(0.074, 0.0, 0.067),
        material="support_gray",
    )
    _add_box(
        support,
        name="center_strut",
        size=(0.018, ROOT_OUTER, 0.039),
        xyz=(0.0, 0.0, 0.0785),
        material="support_gray",
    )
    _add_box(
        support,
        name="root_neck",
        size=(0.020, ROOT_OUTER, 0.030),
        xyz=(0.0, 0.0, 0.044),
        material="support_gray",
    )
    _add_box(
        support,
        name="root_bridge",
        size=(0.028, ROOT_OUTER, 0.014),
        xyz=(0.0, 0.0, 0.022),
        material="support_gray",
    )
    _add_box(
        support,
        name="root_ear_left",
        size=(0.024, root_ear_t, 0.030),
        xyz=(0.0, -root_ear_y, 0.0),
        material="support_gray",
    )
    _add_box(
        support,
        name="root_ear_right",
        size=(0.024, root_ear_t, 0.030),
        xyz=(0.0, root_ear_y, 0.0),
        material="support_gray",
    )

    link_1 = model.part("link_1")
    link1_gap = 0.012
    link1_outer = 0.022
    link1_ear_t = (link1_outer - link1_gap) / 2.0
    link1_ear_y = link1_gap / 2.0 + link1_ear_t / 2.0
    _add_y_cylinder(
        link_1,
        name="prox_eye",
        radius=0.014,
        length=ROOT_GAP,
        xyz=(0.0, 0.0, 0.0),
        material="steel_large",
    )
    _add_box(
        link_1,
        name="shoulder_block",
        size=(0.024, 0.010, 0.010),
        xyz=(0.0, 0.0, -0.019),
        material="steel_large",
    )
    _add_box(
        link_1,
        name="body_upper",
        size=(0.022, 0.010, 0.032),
        xyz=(0.0, 0.0, -0.040),
        material="steel_large",
    )
    _add_box(
        link_1,
        name="body_lower",
        size=(0.018, 0.008, 0.022),
        xyz=(0.0, 0.0, -0.067),
        material="steel_large",
    )
    _add_box(
        link_1,
        name="distal_bridge",
        size=(0.020, link1_outer, 0.014),
        xyz=(0.0, 0.0, -0.084),
        material="steel_large",
    )
    _add_box(
        link_1,
        name="distal_ear_left",
        size=(0.014, link1_ear_t, 0.028),
        xyz=(0.0, -link1_ear_y, -0.105),
        material="steel_large",
    )
    _add_box(
        link_1,
        name="distal_ear_right",
        size=(0.014, link1_ear_t, 0.028),
        xyz=(0.0, link1_ear_y, -0.105),
        material="steel_large",
    )
    _add_box(
        link_1,
        name="ear_web_left",
        size=(0.016, 0.005, 0.010),
        xyz=(0.0, -0.0085, -0.072),
        material="steel_large",
    )
    _add_box(
        link_1,
        name="ear_web_right",
        size=(0.016, 0.005, 0.010),
        xyz=(0.0, 0.0085, -0.072),
        material="steel_large",
    )

    link_2 = model.part("link_2")
    link2_gap = 0.010
    link2_outer = 0.018
    link2_ear_t = (link2_outer - link2_gap) / 2.0
    link2_ear_y = link2_gap / 2.0 + link2_ear_t / 2.0
    _add_y_cylinder(
        link_2,
        name="prox_eye",
        radius=0.012,
        length=link1_gap,
        xyz=(0.0, 0.0, 0.0),
        material="steel_mid",
    )
    _add_box(
        link_2,
        name="shoulder_block",
        size=(0.018, 0.008, 0.008),
        xyz=(0.0, 0.0, -0.016),
        material="steel_mid",
    )
    _add_box(
        link_2,
        name="body_upper",
        size=(0.020, 0.008, 0.024),
        xyz=(0.0, 0.0, -0.032),
        material="steel_mid",
    )
    _add_box(
        link_2,
        name="body_lower",
        size=(0.015, 0.006, 0.020),
        xyz=(0.0, 0.0, -0.054),
        material="steel_mid",
    )
    _add_box(
        link_2,
        name="distal_bridge",
        size=(0.016, link2_outer, 0.012),
        xyz=(0.0, 0.0, -0.064),
        material="steel_mid",
    )
    _add_box(
        link_2,
        name="distal_ear_left",
        size=(0.011, link2_ear_t, 0.024),
        xyz=(0.0, -link2_ear_y, -0.082),
        material="steel_mid",
    )
    _add_box(
        link_2,
        name="distal_ear_right",
        size=(0.011, link2_ear_t, 0.024),
        xyz=(0.0, link2_ear_y, -0.082),
        material="steel_mid",
    )
    _add_box(
        link_2,
        name="ear_web_left",
        size=(0.013, 0.004, 0.008),
        xyz=(0.0, -0.007, -0.056),
        material="steel_mid",
    )
    _add_box(
        link_2,
        name="ear_web_right",
        size=(0.013, 0.004, 0.008),
        xyz=(0.0, 0.007, -0.056),
        material="steel_mid",
    )

    end_link = model.part("end_link")
    _add_y_cylinder(
        end_link,
        name="prox_eye",
        radius=0.010,
        length=link2_gap,
        xyz=(0.0, 0.0, 0.0),
        material="steel_small",
    )
    _add_box(
        end_link,
        name="shoulder_block",
        size=(0.015, 0.006, 0.008),
        xyz=(0.0, 0.0, -0.014),
        material="steel_small",
    )
    _add_box(
        end_link,
        name="body_upper",
        size=(0.015, 0.006, 0.022),
        xyz=(0.0, 0.0, -0.029),
        material="steel_small",
    )
    _add_box(
        end_link,
        name="body_lower",
        size=(0.010, 0.005, 0.020),
        xyz=(0.0, 0.0, -0.050),
        material="steel_small",
    )
    _add_box(
        end_link,
        name="tab_stem",
        size=(0.009, 0.005, 0.018),
        xyz=(0.0, 0.0, -0.069),
        material="steel_small",
    )
    _add_y_cylinder(
        end_link,
        name="end_tab",
        radius=0.0065,
        length=0.005,
        xyz=(0.0, 0.0, -0.081),
        material="steel_small",
    )

    model.articulation(
        "support_to_link_1",
        ArticulationType.REVOLUTE,
        parent=support,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-1.0, upper=1.25),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.0, 0.0, -LINK1_LENGTH)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-1.3, upper=1.3),
    )
    model.articulation(
        "link_2_to_end_link",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=end_link,
        origin=Origin(xyz=(0.0, 0.0, -LINK2_LENGTH)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-1.45, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("bridge_support")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    end_link = object_model.get_part("end_link")

    joint_1 = object_model.get_articulation("support_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")
    joint_3 = object_model.get_articulation("link_2_to_end_link")

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

    ctx.check(
        "three_serial_revolute_joints",
        len(object_model.articulations) == 3
        and all(
            articulation.articulation_type == ArticulationType.REVOLUTE
            for articulation in (joint_1, joint_2, joint_3)
        ),
        "Expected exactly three revolute joints in series.",
    )
    ctx.check(
        "all_joint_axes_pitch_about_y",
        all(tuple(joint.axis) == (0.0, -1.0, 0.0) for joint in (joint_1, joint_2, joint_3)),
        "All three serial joints should pitch about the shared Y-axis.",
    )

    ctx.expect_contact(link_1, support, contact_tol=0.0005, name="root_link_seats_in_bridge_clevis")
    ctx.expect_contact(link_2, link_1, contact_tol=0.0005, name="middle_link_seats_in_first_link_clevis")
    ctx.expect_contact(end_link, link_2, contact_tol=0.0005, name="end_link_seats_in_second_link_clevis")

    def _x_extent(part) -> float | None:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        return aabb[1][0] - aabb[0][0]

    x1 = _x_extent(link_1)
    x2 = _x_extent(link_2)
    x3 = _x_extent(end_link)
    ctx.check(
        "link_sections_taper_toward_distal_tab",
        x1 is not None and x2 is not None and x3 is not None and x1 > x2 > x3,
        f"Expected tapered section widths, got x extents: link_1={x1}, link_2={x2}, end_link={x3}.",
    )

    with ctx.pose({joint_1: 0.45, joint_2: 0.35, joint_3: 0.25}):
        ctx.expect_contact(link_1, support, contact_tol=0.0005, name="root_joint_stays_mounted_in_flexed_pose")
        ctx.expect_contact(link_2, link_1, contact_tol=0.0005, name="middle_joint_stays_mounted_in_flexed_pose")
        ctx.expect_contact(end_link, link_2, contact_tol=0.0005, name="distal_joint_stays_mounted_in_flexed_pose")
        ctx.expect_origin_gap(
            end_link,
            support,
            axis="x",
            min_gap=0.080,
            name="positive_joint_motion_swings_chain_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
