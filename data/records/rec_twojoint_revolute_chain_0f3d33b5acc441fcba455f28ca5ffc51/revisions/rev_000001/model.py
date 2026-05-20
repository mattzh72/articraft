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


def _y_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((0.0, -length / 2.0, 0.0))
    )


def _x_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((-length / 2.0, 0.0, 0.0))
    )


def _make_base_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(150.0, 110.0, 12.0).translate((-86.0, 0.0, -80.0))
    pedestal = cq.Workplane("XY").box(44.0, 60.0, 58.0).translate((-54.0, 0.0, -45.0))
    tie = cq.Workplane("XY").box(16.0, 20.0, 8.0).translate((-22.0, 0.0, -26.0))
    pos_gusset = cq.Workplane("XY").box(30.0, 8.0, 24.0).translate((-22.0, 14.0, -22.0))
    neg_gusset = cq.Workplane("XY").box(30.0, 8.0, 24.0).translate((-22.0, -14.0, -22.0))
    pos_ear = cq.Workplane("XY").box(18.0, 8.0, 34.0).translate((0.0, 20.0, 0.0))
    neg_ear = cq.Workplane("XY").box(18.0, 8.0, 34.0).translate((0.0, -20.0, 0.0))
    pos_boss = _y_cylinder(15.0, 4.0).translate((0.0, 26.0, 0.0))
    neg_boss = _y_cylinder(15.0, 4.0).translate((0.0, -26.0, 0.0))

    base = (
        plate.union(pedestal)
        .union(tie)
        .union(pos_gusset)
        .union(neg_gusset)
        .union(pos_ear)
        .union(neg_ear)
        .union(pos_boss)
        .union(neg_boss)
    )

    hinge_bore = _y_cylinder(8.2, 64.0)
    mount_holes = (
        cq.Workplane("XY")
        .pushPoints([(-124.0, -32.0), (-124.0, 32.0), (-60.0, -32.0), (-60.0, 32.0)])
        .circle(5.0)
        .extrude(30.0)
        .translate((0.0, 0.0, -94.0))
    )
    return base.cut(hinge_bore).cut(mount_holes)


def _make_primary_shape() -> cq.Workplane:
    rear_lug = _y_cylinder(12.0, 20.0)
    root_block = cq.Workplane("XY").box(26.0, 18.0, 22.0).translate((18.0, 0.0, 0.0))
    beam = cq.Workplane("XY").box(184.0, 32.0, 24.0).translate((118.0, 0.0, 0.0))
    fork_root = cq.Workplane("XY").box(24.0, 18.0, 16.0).translate((228.0, 0.0, 0.0))
    pos_fork = cq.Workplane("XY").box(72.0, 8.0, 24.0).translate((266.0, 20.0, 0.0))
    neg_fork = cq.Workplane("XY").box(72.0, 8.0, 24.0).translate((266.0, -20.0, 0.0))
    pos_fork_rib = cq.Workplane("XY").box(26.0, 8.0, 18.0).translate((236.0, 14.0, 0.0))
    neg_fork_rib = cq.Workplane("XY").box(26.0, 8.0, 18.0).translate((236.0, -14.0, 0.0))
    pos_boss = _y_cylinder(12.0, 4.0).translate((290.0, 26.0, 0.0))
    neg_boss = _y_cylinder(12.0, 4.0).translate((290.0, -26.0, 0.0))

    primary = (
        rear_lug.union(root_block)
        .union(beam)
        .union(fork_root)
        .union(pos_fork)
        .union(neg_fork)
        .union(pos_fork_rib)
        .union(neg_fork_rib)
        .union(pos_boss)
        .union(neg_boss)
    )

    shoulder_bore = _y_cylinder(8.2, 36.0)
    elbow_bore = _y_cylinder(6.8, 64.0).translate((290.0, 0.0, 0.0))
    return primary.cut(shoulder_bore).cut(elbow_bore)


def _make_secondary_shape() -> cq.Workplane:
    rear_lug = _y_cylinder(10.0, 18.0)
    root_web = cq.Workplane("XY").box(18.0, 12.0, 16.0).translate((12.0, 0.0, 0.0))
    beam = cq.Workplane("XY").box(144.0, 18.0, 20.0).translate((90.0, 0.0, 0.0))
    nose_block = cq.Workplane("XY").box(20.0, 24.0, 24.0).translate((168.0, 0.0, 0.0))
    tool_tab = cq.Workplane("XY").box(12.0, 42.0, 42.0).translate((198.0, 0.0, 0.0))
    upper_nose_rib = (
        cq.Workplane("XZ")
        .moveTo(156.0, 10.0)
        .lineTo(184.0, 18.0)
        .lineTo(188.0, 18.0)
        .lineTo(188.0, 10.0)
        .close()
        .extrude(8.0)
        .translate((0.0, -4.0, 0.0))
    )
    lower_nose_rib = (
        cq.Workplane("XZ")
        .moveTo(156.0, -10.0)
        .lineTo(188.0, -10.0)
        .lineTo(188.0, -18.0)
        .lineTo(184.0, -18.0)
        .close()
        .extrude(8.0)
        .translate((0.0, -4.0, 0.0))
    )

    secondary = (
        rear_lug.union(root_web)
        .union(beam)
        .union(nose_block)
        .union(tool_tab)
        .union(upper_nose_rib)
        .union(lower_nose_rib)
    )

    elbow_bore = _y_cylinder(6.8, 32.0)
    tool_hole = _x_cylinder(7.0, 20.0).translate((198.0, 0.0, 0.0))
    return secondary.cut(elbow_bore).cut(tool_hole)


def _base_components() -> dict[str, cq.Workplane]:
    plate = cq.Workplane("XY").box(150.0, 110.0, 12.0).translate((-86.0, 0.0, -80.0))
    pedestal = cq.Workplane("XY").box(44.0, 60.0, 58.0).translate((-54.0, 0.0, -45.0))
    tie = cq.Workplane("XY").box(16.0, 20.0, 8.0).translate((-22.0, 0.0, -26.0))
    body = plate.union(pedestal).union(tie)

    joint = (
        cq.Workplane("XY").box(30.0, 8.0, 24.0).translate((-22.0, 14.0, -22.0))
        .union(cq.Workplane("XY").box(30.0, 8.0, 24.0).translate((-22.0, -14.0, -22.0)))
        .union(cq.Workplane("XY").box(18.0, 8.0, 34.0).translate((0.0, 20.0, 0.0)))
        .union(cq.Workplane("XY").box(18.0, 8.0, 34.0).translate((0.0, -20.0, 0.0)))
        .union(_y_cylinder(15.0, 4.0).translate((0.0, 26.0, 0.0)))
        .union(_y_cylinder(15.0, 4.0).translate((0.0, -26.0, 0.0)))
    )

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints([(-124.0, -32.0), (-124.0, 32.0), (-60.0, -32.0), (-60.0, 32.0)])
        .circle(5.0)
        .extrude(30.0)
        .translate((0.0, 0.0, -94.0))
    )
    hinge_bore = _y_cylinder(8.2, 64.0)
    return {
        "body": body.cut(mount_holes),
        "joint": joint.cut(hinge_bore),
    }


def _primary_components() -> dict[str, cq.Workplane]:
    shoulder_lug = (
        _y_cylinder(12.0, 20.0)
        .union(cq.Workplane("XY").box(26.0, 18.0, 22.0).translate((18.0, 0.0, 0.0)))
        .cut(_y_cylinder(8.2, 36.0))
    )
    arm_body = cq.Workplane("XY").box(184.0, 32.0, 24.0).translate((118.0, 0.0, 0.0))
    elbow_fork = (
        cq.Workplane("XY").box(24.0, 18.0, 16.0).translate((218.0, 0.0, 0.0))
        .union(cq.Workplane("XY").box(24.0, 18.0, 16.0).translate((228.0, 0.0, 0.0)))
        .union(cq.Workplane("XY").box(72.0, 8.0, 24.0).translate((266.0, 20.0, 0.0)))
        .union(cq.Workplane("XY").box(72.0, 8.0, 24.0).translate((266.0, -20.0, 0.0)))
        .union(cq.Workplane("XY").box(26.0, 8.0, 18.0).translate((236.0, 14.0, 0.0)))
        .union(cq.Workplane("XY").box(26.0, 8.0, 18.0).translate((236.0, -14.0, 0.0)))
        .union(_y_cylinder(12.0, 4.0).translate((290.0, 26.0, 0.0)))
        .union(_y_cylinder(12.0, 4.0).translate((290.0, -26.0, 0.0)))
        .cut(_y_cylinder(6.8, 64.0).translate((290.0, 0.0, 0.0)))
    )
    return {
        "shoulder_lug": shoulder_lug,
        "arm_body": arm_body,
        "elbow_fork": elbow_fork,
    }


def _secondary_components() -> dict[str, cq.Workplane]:
    elbow_lug = (
        _y_cylinder(10.0, 18.0)
        .union(cq.Workplane("XY").box(18.0, 12.0, 16.0).translate((12.0, 0.0, 0.0)))
        .cut(_y_cylinder(6.8, 32.0))
    )
    arm = (
        cq.Workplane("XY").box(144.0, 18.0, 20.0).translate((90.0, 0.0, 0.0))
        .union(cq.Workplane("XY").box(20.0, 24.0, 24.0).translate((168.0, 0.0, 0.0)))
        .union(cq.Workplane("XY").box(12.0, 42.0, 42.0).translate((198.0, 0.0, 0.0)))
        .union(
            cq.Workplane("XZ")
            .moveTo(156.0, 10.0)
            .lineTo(184.0, 18.0)
            .lineTo(188.0, 18.0)
            .lineTo(188.0, 10.0)
            .close()
            .extrude(8.0)
            .translate((0.0, -4.0, 0.0))
        )
        .union(
            cq.Workplane("XZ")
            .moveTo(156.0, -10.0)
            .lineTo(188.0, -10.0)
            .lineTo(188.0, -18.0)
            .lineTo(184.0, -18.0)
            .close()
            .extrude(8.0)
            .translate((0.0, -4.0, 0.0))
        )
        .cut(_x_cylinder(7.0, 20.0).translate((198.0, 0.0, 0.0)))
    )
    return {
        "elbow_lug": elbow_lug,
        "arm_body": arm,
    }


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_bench_arm")

    model.material("painted_steel", rgba=(0.27, 0.30, 0.33, 1.0))
    model.material("machined_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    model.material("tool_gray", rgba=(0.52, 0.55, 0.58, 1.0))

    base = model.part("base")
    base_components = _base_components()
    base.visual(
        mesh_from_cadquery(base_components["body"], "base_body", unit_scale=0.001),
        material="painted_steel",
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(base_components["joint"], "base_joint", unit_scale=0.001),
        material="machined_steel",
        name="base_joint",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.18, 0.12, 0.09)),
        mass=4.5,
        origin=Origin(xyz=(0.035, 0.0, -0.045)),
    )

    primary = model.part("primary_link")
    primary_components = _primary_components()
    primary.visual(
        mesh_from_cadquery(primary_components["shoulder_lug"], "primary_shoulder_lug", unit_scale=0.001),
        material="machined_steel",
        name="shoulder_lug",
    )
    primary.visual(
        mesh_from_cadquery(primary_components["arm_body"], "primary_arm_body", unit_scale=0.001),
        material="machined_steel",
        name="arm_body",
    )
    primary.visual(
        mesh_from_cadquery(primary_components["elbow_fork"], "primary_elbow_fork", unit_scale=0.001),
        material="machined_steel",
        name="elbow_fork",
    )
    primary.inertial = Inertial.from_geometry(
        Box((0.31, 0.06, 0.05)),
        mass=2.3,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
    )

    secondary = model.part("secondary_link")
    secondary_components = _secondary_components()
    secondary.visual(
        mesh_from_cadquery(secondary_components["elbow_lug"], "secondary_elbow_lug", unit_scale=0.001),
        material="machined_steel",
        name="elbow_lug",
    )
    secondary.visual(
        mesh_from_cadquery(secondary_components["arm_body"], "secondary_arm_body", unit_scale=0.001),
        material="tool_gray",
        name="arm_body",
    )
    secondary.inertial = Inertial.from_geometry(
        Box((0.23, 0.05, 0.05)),
        mass=1.2,
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_primary",
        ArticulationType.REVOLUTE,
        parent=base,
        child=primary,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.5,
            lower=-0.30,
            upper=1.15,
        ),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(0.29, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.0,
            lower=-0.55,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    primary = object_model.get_part("primary_link")
    secondary = object_model.get_part("secondary_link")
    shoulder = object_model.get_articulation("base_to_primary")
    elbow = object_model.get_articulation("primary_to_secondary")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        base,
        primary,
        elem_a="base_joint",
        elem_b="shoulder_lug",
        reason="nested shoulder clevis and lug share coaxial support volume without an explicit pin part",
    )
    ctx.allow_overlap(
        primary,
        secondary,
        elem_a="elbow_fork",
        elem_b="elbow_lug",
        reason="nested elbow fork and lug share coaxial support volume without an explicit pin part",
    )

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
        "parallel_supported_joint_axes",
        shoulder.axis == (0.0, 1.0, 0.0) and elbow.axis == (0.0, 1.0, 0.0),
        details=(
            f"expected both serial hinges on +Y, got shoulder={shoulder.axis}, "
            f"elbow={elbow.axis}"
        ),
    )
    ctx.check(
        "shoulder_to_elbow_spacing",
        elbow.origin.xyz == (0.29, 0.0, 0.0),
        details=f"expected elbow hinge at 0.29 m along primary link, got {elbow.origin.xyz}",
    )
    ctx.expect_contact(
        primary,
        base,
        contact_tol=5e-4,
        elem_a="shoulder_lug",
        elem_b="base_joint",
        name="shoulder_joint_has_captured_support_contact",
    )
    ctx.expect_contact(
        secondary,
        primary,
        contact_tol=5e-4,
        elem_a="elbow_lug",
        elem_b="elbow_fork",
        name="elbow_joint_has_captured_support_contact",
    )
    ctx.expect_gap(
        primary,
        base,
        axis="x",
        positive_elem="arm_body",
        negative_elem="base_body",
        min_gap=0.005,
        name="primary_arm_clears_base_structure",
    )
    ctx.expect_gap(
        secondary,
        primary,
        axis="x",
        positive_elem="arm_body",
        negative_elem="elbow_fork",
        min_gap=0.004,
        name="secondary_arm_starts_forward_of_elbow_fork",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
