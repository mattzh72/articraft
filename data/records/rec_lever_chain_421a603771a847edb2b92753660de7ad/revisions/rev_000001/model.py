from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SUPPORT_GAP = 0.020
LINK1_THICKNESS = SUPPORT_GAP
LINK2_THICKNESS = 0.014
END_TAB_THICKNESS = 0.008

LINK1_LENGTH = 0.115
LINK2_LENGTH = 0.090
END_TAB_LENGTH = 0.062


def _extruded_circle_on_xz(x_pos: float, z_pos: float, radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x_pos, z_pos)
        .circle(radius)
        .extrude(thickness)
        .translate((0.0, thickness / 2.0, 0.0))
    )


def _extruded_polygon_on_xz(points: list[tuple[float, float]], thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((0.0, thickness / 2.0, 0.0))
    )


def _cylindrical_y_cut(z_pos: float, radius: float, thickness: float) -> cq.Workplane:
    cut_len = thickness + 0.008
    return (
        cq.Workplane("XZ")
        .center(0.0, z_pos)
        .circle(radius)
        .extrude(cut_len)
        .translate((0.0, cut_len / 2.0, 0.0))
    )


def _open_fork_cut(
    *,
    z_center: float,
    outer_radius: float,
    slot_width: float,
    slot_x: float,
    child_outer_radius: float,
) -> cq.Workplane:
    slot_top = z_center + child_outer_radius + 0.003
    slot_bottom = z_center - outer_radius - 0.004
    return cq.Workplane("XY").box(
        slot_x,
        slot_width,
        slot_top - slot_bottom,
        centered=(True, True, True),
    ).translate((0.0, 0.0, (slot_top + slot_bottom) / 2.0))


def _make_fork_link(
    *,
    length: float,
    thickness: float,
    prox_outer_radius: float,
    prox_hole_radius: float,
    dist_outer_radius: float,
    dist_hole_radius: float,
    top_web_width: float,
    bottom_web_width: float,
    distal_slot_width: float,
    child_outer_radius: float,
) -> cq.Workplane:
    proximal_boss = _extruded_circle_on_xz(0.0, 0.0, prox_outer_radius, thickness)
    distal_boss = _extruded_circle_on_xz(0.0, -length, dist_outer_radius, thickness)
    web = _extruded_polygon_on_xz(
        [
            (-top_web_width / 2.0, -prox_outer_radius * 0.35),
            (top_web_width / 2.0, -prox_outer_radius * 0.35),
            (bottom_web_width / 2.0, -length + dist_outer_radius * 0.55),
            (-bottom_web_width / 2.0, -length + dist_outer_radius * 0.55),
        ],
        thickness,
    )

    distal_pin = _extruded_circle_on_xz(0.0, -length, dist_hole_radius, thickness)

    body = proximal_boss.union(distal_boss).union(web)
    body = body.cut(_cylindrical_y_cut(0.0, prox_hole_radius, thickness))
    body = body.cut(
        _open_fork_cut(
            z_center=-length,
            outer_radius=dist_outer_radius,
            slot_width=distal_slot_width + 0.002,
            slot_x=2.0 * dist_outer_radius + 0.006,
            child_outer_radius=child_outer_radius,
        )
    )
    return body.union(distal_pin)


def _make_end_tab() -> cq.Workplane:
    prox_outer_radius = 0.0105
    prox_hole_radius = 0.0045
    distal_pad_radius = 0.0065
    distal_hole_radius = 0.0027
    distal_hole_z = -0.050

    proximal_boss = _extruded_circle_on_xz(0.0, 0.0, prox_outer_radius, END_TAB_THICKNESS)
    distal_pad = _extruded_circle_on_xz(0.0, distal_hole_z, distal_pad_radius, END_TAB_THICKNESS)
    shank = _extruded_polygon_on_xz(
        [
            (-0.0060, -prox_outer_radius * 0.30),
            (0.0060, -prox_outer_radius * 0.30),
            (0.0045, -END_TAB_LENGTH + 0.012),
            (-0.0045, -END_TAB_LENGTH + 0.012),
        ],
        END_TAB_THICKNESS,
    )
    end_blade = cq.Workplane("XY").box(
        0.014,
        END_TAB_THICKNESS,
        0.016,
        centered=(True, True, True),
    ).translate((0.0, 0.0, -END_TAB_LENGTH + 0.008))

    body = proximal_boss.union(distal_pad).union(shank).union(end_blade)
    body = body.cut(_cylindrical_y_cut(0.0, prox_hole_radius, END_TAB_THICKNESS))
    body = body.cut(_cylindrical_y_cut(distal_hole_z, distal_hole_radius, END_TAB_THICKNESS))
    return body


def _make_bridge_support() -> cq.Workplane:
    top_bridge = cq.Workplane("XY").box(0.160, 0.036, 0.014, centered=(True, True, True)).translate(
        (0.0, 0.0, 0.095)
    )
    left_leg = cq.Workplane("XY").box(0.022, 0.024, 0.072, centered=(True, True, True)).translate(
        (-0.055, 0.0, 0.052)
    )
    right_leg = cq.Workplane("XY").box(0.022, 0.024, 0.072, centered=(True, True, True)).translate(
        (0.055, 0.0, 0.052)
    )
    lower_bridge = cq.Workplane("XY").box(0.132, 0.018, 0.010, centered=(True, True, True)).translate(
        (0.0, 0.0, 0.025)
    )
    center_stem = cq.Workplane("XY").box(0.022, 0.024, 0.076, centered=(True, True, True)).translate(
        (0.0, 0.0, 0.051)
    )
    clevis_block = cq.Workplane("XY").box(0.050, 0.030, 0.046, centered=(True, True, True)).translate(
        (0.0, 0.0, 0.007)
    )

    support = top_bridge.union(left_leg).union(right_leg).union(lower_bridge).union(center_stem).union(clevis_block)

    root_pin = _extruded_circle_on_xz(0.0, 0.0, 0.0075, 0.030)
    root_slot_top = 0.018 + 0.003
    root_slot_bottom = -0.015 - 0.004
    root_slot = cq.Workplane("XY").box(
        0.042,
        SUPPORT_GAP + 0.002,
        root_slot_top - root_slot_bottom,
        centered=(True, True, True),
    ).translate((0.0, 0.0, (root_slot_top + root_slot_bottom) / 2.0))
    bridge_mount_holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.050, 0.0), (0.050, 0.0)])
        .circle(0.0045)
        .extrude(0.030)
        .translate((0.0, 0.0, 0.080))
    )

    return support.cut(root_slot).cut(bridge_mount_holes).union(root_pin)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_lever_chain")

    model.material("bridge_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("lever_root", rgba=(0.62, 0.65, 0.70, 1.0))
    model.material("lever_mid", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("lever_tip", rgba=(0.82, 0.84, 0.87, 1.0))

    bridge_support = model.part("bridge_support")
    bridge_support.visual(
        mesh_from_cadquery(_make_bridge_support(), "bridge_support"),
        material="bridge_steel",
        name="support_body",
    )
    bridge_support.inertial = Inertial.from_geometry(
        Box((0.160, 0.036, 0.118)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
    )

    primary_link = model.part("primary_link")
    primary_link.visual(
        mesh_from_cadquery(
            _make_fork_link(
                length=LINK1_LENGTH,
                thickness=LINK1_THICKNESS,
                prox_outer_radius=0.018,
                prox_hole_radius=0.0075,
                dist_outer_radius=0.015,
                dist_hole_radius=0.0060,
                top_web_width=0.024,
                bottom_web_width=0.020,
                distal_slot_width=LINK2_THICKNESS,
                child_outer_radius=0.013,
            ),
            "primary_link",
        ),
        material="lever_root",
        name="primary_link_body",
    )
    primary_link.inertial = Inertial.from_geometry(
        Box((0.038, LINK1_THICKNESS, 0.132)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, -LINK1_LENGTH / 2.0)),
    )

    secondary_link = model.part("secondary_link")
    secondary_link.visual(
        mesh_from_cadquery(
            _make_fork_link(
                length=LINK2_LENGTH,
                thickness=LINK2_THICKNESS,
                prox_outer_radius=0.013,
                prox_hole_radius=0.0060,
                dist_outer_radius=0.0105,
                dist_hole_radius=0.0045,
                top_web_width=0.018,
                bottom_web_width=0.013,
                distal_slot_width=END_TAB_THICKNESS,
                child_outer_radius=0.0105,
            ),
            "secondary_link",
        ),
        material="lever_mid",
        name="secondary_link_body",
    )
    secondary_link.inertial = Inertial.from_geometry(
        Box((0.028, LINK2_THICKNESS, 0.104)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -LINK2_LENGTH / 2.0)),
    )

    end_tab = model.part("end_tab")
    end_tab.visual(
        mesh_from_cadquery(_make_end_tab(), "end_tab"),
        material="lever_tip",
        name="end_tab_body",
    )
    end_tab.inertial = Inertial.from_geometry(
        Box((0.021, END_TAB_THICKNESS, 0.068)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, -END_TAB_LENGTH / 2.0)),
    )

    model.articulation(
        "support_to_primary",
        ArticulationType.REVOLUTE,
        parent=bridge_support,
        child=primary_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.35, upper=1.35, effort=18.0, velocity=2.0),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(0.0, 0.0, -LINK1_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.55, upper=1.55, effort=12.0, velocity=2.4),
    )
    model.articulation(
        "secondary_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=end_tab,
        origin=Origin(xyz=(0.0, 0.0, -LINK2_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.65, upper=1.65, effort=6.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    def _extent(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis] - aabb[0][axis]

    def _center_axis(
        aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
        axis: int,
    ) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][axis] + aabb[1][axis]) / 2.0

    ctx = TestContext(object_model)
    bridge_support = object_model.get_part("bridge_support")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    end_tab = object_model.get_part("end_tab")

    root_joint = object_model.get_articulation("support_to_primary")
    middle_joint = object_model.get_articulation("primary_to_secondary")
    distal_joint = object_model.get_articulation("secondary_to_end_tab")

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
    ctx.allow_overlap(
        bridge_support,
        primary_link,
        reason="The root clevis is modeled as a retained pin-and-eye journal fit with the primary link captured on the support pin.",
    )
    ctx.allow_overlap(
        primary_link,
        secondary_link,
        reason="The first moving joint is intentionally represented as a captured fork-and-eye revolute nest around an integrated pin.",
    )
    ctx.allow_overlap(
        secondary_link,
        end_tab,
        reason="The distal small tab is intentionally represented as a retained revolute eye around the secondary link pin.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        bridge_support,
        primary_link,
        name="bridge clevis supports the primary link eye",
    )
    ctx.expect_contact(
        primary_link,
        secondary_link,
        name="primary fork supports the secondary link eye",
    )
    ctx.expect_contact(
        secondary_link,
        end_tab,
        name="secondary fork supports the end tab eye",
    )

    primary_aabb = ctx.part_world_aabb(primary_link)
    secondary_aabb = ctx.part_world_aabb(secondary_link)
    end_tab_aabb = ctx.part_world_aabb(end_tab)
    primary_dx = _extent(primary_aabb, 0)
    secondary_dx = _extent(secondary_aabb, 0)
    end_tab_dx = _extent(end_tab_aabb, 0)
    primary_dz = _extent(primary_aabb, 2)
    secondary_dz = _extent(secondary_aabb, 2)
    end_tab_dz = _extent(end_tab_aabb, 2)
    ctx.check(
        "link sections taper toward the distal tab",
        all(value is not None for value in (primary_dx, secondary_dx, end_tab_dx, primary_dz, secondary_dz, end_tab_dz))
        and primary_dx > secondary_dx > end_tab_dx
        and primary_dz > secondary_dz > end_tab_dz,
        details=(
            f"dx=({primary_dx}, {secondary_dx}, {end_tab_dx}), "
            f"dz=({primary_dz}, {secondary_dz}, {end_tab_dz})"
        ),
    )

    secondary_rest = ctx.part_world_position(secondary_link)
    with ctx.pose({root_joint: 0.55}):
        secondary_swung = ctx.part_world_position(secondary_link)
    ctx.check(
        "root revolute swings the linkage laterally",
        secondary_rest is not None
        and secondary_swung is not None
        and secondary_swung[0] < secondary_rest[0] - 0.045
        and secondary_swung[2] > secondary_rest[2] + 0.010,
        details=f"rest={secondary_rest}, swung={secondary_swung}",
    )

    end_rest = ctx.part_world_position(end_tab)
    with ctx.pose({middle_joint: 0.60}):
        end_mid_bent = ctx.part_world_position(end_tab)
    ctx.check(
        "middle revolute folds the distal stages",
        end_rest is not None
        and end_mid_bent is not None
        and end_mid_bent[0] < end_rest[0] - 0.040
        and end_mid_bent[2] > end_rest[2] + 0.012,
        details=f"rest={end_rest}, bent={end_mid_bent}",
    )

    tab_rest = ctx.part_element_world_aabb(end_tab, elem="end_tab_body")
    with ctx.pose({distal_joint: 0.70}):
        tab_rotated = ctx.part_element_world_aabb(end_tab, elem="end_tab_body")
    tab_rest_x = _center_axis(tab_rest, 0)
    tab_rotated_x = _center_axis(tab_rotated, 0)
    ctx.check(
        "distal revolute reorients the small end tab",
        tab_rest_x is not None and tab_rotated_x is not None and tab_rotated_x < tab_rest_x - 0.010,
        details=f"rest_center_x={tab_rest_x}, rotated_center_x={tab_rotated_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
