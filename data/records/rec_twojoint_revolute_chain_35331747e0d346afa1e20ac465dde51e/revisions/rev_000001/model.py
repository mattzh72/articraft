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


ROOT_AXIS = (0.0, -1.0, 0.0)
ELBOW_AXIS = (0.0, -1.0, 0.0)
ELBOW_DROP = 0.165


def _rounded_rect_sketch(width: float, thickness: float, radius: float) -> cq.Sketch:
    return (
        cq.Sketch()
        .rect(width, thickness)
        .vertices()
        .fillet(min(radius, width * 0.24, thickness * 0.24))
    )


def _lofted_bar(
    *,
    top_width: float,
    top_thickness: float,
    bottom_width: float,
    bottom_thickness: float,
    z_top: float,
    z_bottom: float,
    top_radius: float,
    bottom_radius: float,
) -> cq.Workplane:
    top = _rounded_rect_sketch(top_width, top_thickness, top_radius).moved(
        cq.Location(cq.Vector(0.0, 0.0, z_top))
    )
    bottom = _rounded_rect_sketch(bottom_width, bottom_thickness, bottom_radius).moved(
        cq.Location(cq.Vector(0.0, 0.0, z_bottom))
    )
    return cq.Workplane("XY").placeSketch(top, bottom).loft(combine=True)


def _fuse_all(*solids: cq.Workplane) -> cq.Workplane:
    fused = solids[0]
    for solid in solids[1:]:
        fused = fused.union(solid)
    return fused


def _make_bridge_support() -> cq.Workplane:
    root_gap = 0.016
    cheek_thickness = 0.008

    left_foot = cq.Workplane("XY").box(0.072, 0.086, 0.015).translate((-0.085, 0.0, -0.2425))
    right_foot = cq.Workplane("XY").box(0.072, 0.086, 0.015).translate((0.085, 0.0, -0.2425))
    left_upright = cq.Workplane("XY").box(0.032, 0.050, 0.290).translate((-0.085, 0.0, -0.090))
    right_upright = cq.Workplane("XY").box(0.032, 0.050, 0.290).translate((0.085, 0.0, -0.090))

    top_beam = cq.Workplane("XY").box(0.222, 0.052, 0.030).translate((0.0, 0.0, 0.055))
    mid_tie = cq.Workplane("XY").box(0.124, 0.024, 0.018).translate((0.0, 0.0, -0.055))

    hanger_cap = cq.Workplane("XY").box(0.042, 0.036, 0.020).translate((0.0, 0.0, 0.043))
    left_cheek = cq.Workplane("XY").box(0.038, cheek_thickness, 0.052).translate(
        (0.0, root_gap * 0.5 + cheek_thickness * 0.5, 0.009)
    )
    right_cheek = cq.Workplane("XY").box(0.038, cheek_thickness, 0.052).translate(
        (0.0, -(root_gap * 0.5 + cheek_thickness * 0.5), 0.009)
    )

    bridge = _fuse_all(
        left_foot,
        right_foot,
        left_upright,
        right_upright,
        top_beam,
        mid_tie,
        hanger_cap,
        left_cheek,
        right_cheek,
    )
    return bridge.edges("|Z").fillet(0.0035)


def _make_proximal_link_body() -> cq.Workplane:
    proximal_barrel = cq.Workplane("XZ").circle(0.014).extrude(0.008, both=True)
    neck = cq.Workplane("XY").box(0.024, 0.016, 0.026).translate((0.0, 0.0, -0.013))
    beam = _lofted_bar(
        top_width=0.030,
        top_thickness=0.018,
        bottom_width=0.022,
        bottom_thickness=0.014,
        z_top=-0.024,
        z_bottom=-0.146,
        top_radius=0.0035,
        bottom_radius=0.0025,
    )
    return _fuse_all(proximal_barrel, neck, beam)


def _make_distal_link_main() -> cq.Workplane:
    proximal_barrel = cq.Workplane("XZ").circle(0.010).extrude(0.006, both=True)
    neck = cq.Workplane("XY").box(0.018, 0.012, 0.020).translate((0.0, 0.0, -0.010))
    beam = _lofted_bar(
        top_width=0.020,
        top_thickness=0.013,
        bottom_width=0.014,
        bottom_thickness=0.009,
        z_top=-0.018,
        z_bottom=-0.104,
        top_radius=0.0025,
        bottom_radius=0.0018,
    )
    return _fuse_all(proximal_barrel, neck, beam)


def _make_end_tab() -> cq.Workplane:
    tab = cq.Workplane("XY").box(0.016, 0.008, 0.038).translate((0.0, 0.0, -0.121))
    hole = cq.Workplane("XZ").center(0.0, -0.130).circle(0.0038).extrude(0.014, both=True)
    tab = tab.cut(hole)
    return tab.edges("|Y").fillet(0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_mounted_revolute_chain")

    dark_frame = model.material("dark_frame", color=(0.20, 0.22, 0.25, 1.0))
    satin_link = model.material("satin_link", color=(0.68, 0.71, 0.76, 1.0))
    end_tab_finish = model.material("end_tab_finish", color=(0.62, 0.64, 0.68, 1.0))

    bridge_support = model.part("bridge_support")
    bridge_support.visual(
        Box((0.072, 0.086, 0.015)),
        origin=Origin(xyz=(-0.085, 0.0, -0.2425)),
        material=dark_frame,
        name="left_foot",
    )
    bridge_support.visual(
        Box((0.072, 0.086, 0.015)),
        origin=Origin(xyz=(0.085, 0.0, -0.2425)),
        material=dark_frame,
        name="right_foot",
    )
    bridge_support.visual(
        Box((0.032, 0.050, 0.275)),
        origin=Origin(xyz=(-0.085, 0.0, -0.0975)),
        material=dark_frame,
        name="left_upright",
    )
    bridge_support.visual(
        Box((0.032, 0.050, 0.275)),
        origin=Origin(xyz=(0.085, 0.0, -0.0975)),
        material=dark_frame,
        name="right_upright",
    )
    bridge_support.visual(
        Box((0.222, 0.052, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=dark_frame,
        name="top_beam",
    )
    bridge_support.visual(
        Box((0.036, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=dark_frame,
        name="hanger_block",
    )
    bridge_support.visual(
        Box((0.034, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, 0.012, 0.005)),
        material=dark_frame,
        name="left_root_cheek",
    )
    bridge_support.visual(
        Box((0.034, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, -0.012, 0.005)),
        material=dark_frame,
        name="right_root_cheek",
    )
    bridge_support.visual(
        Box((0.026, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=dark_frame,
        name="center_cap",
    )
    bridge_support.visual(
        Box((0.028, 0.010, 0.024)),
        origin=Origin(xyz=(-0.020, 0.015, 0.028)),
        material=dark_frame,
        name="left_gusset",
    )
    bridge_support.visual(
        Box((0.028, 0.010, 0.024)),
        origin=Origin(xyz=(0.020, -0.015, 0.028)),
        material=dark_frame,
        name="right_gusset",
    )
    bridge_support.visual(
        Box((0.028, 0.010, 0.020)),
        origin=Origin(xyz=(-0.020, -0.015, 0.033)),
        material=dark_frame,
        name="left_rear_tie",
    )
    bridge_support.visual(
        Box((0.028, 0.010, 0.020)),
        origin=Origin(xyz=(0.020, 0.015, 0.033)),
        material=dark_frame,
        name="right_rear_tie",
    )
    bridge_support.visual(
        Box((0.036, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=dark_frame,
        name="support_frame",
    )

    proximal_link = model.part("proximal_link")
    proximal_link.visual(
        mesh_from_cadquery(_make_proximal_link_body(), "proximal_link_body"),
        origin=Origin(),
        material=satin_link,
        name="link_body",
    )
    proximal_link.visual(
        Box((0.028, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, 0.009, -0.164)),
        material=satin_link,
        name="left_distal_cheek",
    )
    proximal_link.visual(
        Box((0.028, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, -0.009, -0.164)),
        material=satin_link,
        name="right_distal_cheek",
    )

    distal_link = model.part("distal_link")
    distal_link.visual(
        mesh_from_cadquery(_make_distal_link_main(), "distal_link_main"),
        origin=Origin(),
        material=satin_link,
        name="link_body",
    )
    distal_link.visual(
        mesh_from_cadquery(_make_end_tab(), "distal_end_tab"),
        origin=Origin(),
        material=end_tab_finish,
        name="end_tab",
    )

    model.articulation(
        "bridge_to_proximal",
        ArticulationType.REVOLUTE,
        parent=bridge_support,
        child=proximal_link,
        origin=Origin(),
        axis=ROOT_AXIS,
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-1.05,
            upper=1.20,
        ),
    )
    model.articulation(
        "proximal_to_distal",
        ArticulationType.REVOLUTE,
        parent=proximal_link,
        child=distal_link,
        origin=Origin(xyz=(0.0, 0.0, -ELBOW_DROP)),
        axis=ELBOW_AXIS,
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-1.25,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge_support = object_model.get_part("bridge_support")
    proximal_link = object_model.get_part("proximal_link")
    distal_link = object_model.get_part("distal_link")
    root_joint = object_model.get_articulation("bridge_to_proximal")
    elbow_joint = object_model.get_articulation("proximal_to_distal")
    end_tab = distal_link.get_visual("end_tab")

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
        "part_and_joint_names_resolve",
        all(
            item is not None
            for item in (
                bridge_support,
                proximal_link,
                distal_link,
                root_joint,
                elbow_joint,
                end_tab,
            )
        ),
        "Expected bridge support, both chain links, both joints, and the distal end tab visual.",
    )
    ctx.check(
        "parallel_revolute_axes",
        root_joint.axis == ROOT_AXIS and elbow_joint.axis == ELBOW_AXIS,
        f"Expected both joint axes to be {ROOT_AXIS}, got {root_joint.axis} and {elbow_joint.axis}.",
    )

    ctx.expect_contact(
        bridge_support,
        proximal_link,
        name="root_clevis_supports_proximal_link",
    )
    ctx.expect_contact(
        proximal_link,
        distal_link,
        name="distal_clevis_supports_distal_link",
    )
    ctx.expect_origin_distance(
        bridge_support,
        proximal_link,
        axes="xyz",
        max_dist=1e-6,
        name="root_joint_frames_coincident_at_rest",
    )
    ctx.expect_origin_gap(
        proximal_link,
        distal_link,
        axis="z",
        min_gap=0.160,
        max_gap=0.170,
        name="second_joint_sits_below_root_joint",
    )

    with ctx.pose({root_joint: 0.78}):
        ctx.expect_gap(
            distal_link,
            bridge_support,
            axis="x",
            positive_elem="end_tab",
            min_gap=0.035,
            name="root_joint_positive_motion_swings_chain_forward",
        )

    with ctx.pose({root_joint: 0.0, elbow_joint: 0.82}):
        ctx.expect_gap(
            distal_link,
            proximal_link,
            axis="x",
            positive_elem="end_tab",
            min_gap=0.030,
            name="elbow_joint_positive_motion_swings_end_tab_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
