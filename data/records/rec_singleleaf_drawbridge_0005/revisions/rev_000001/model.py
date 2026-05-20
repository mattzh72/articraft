from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_singleleaf_drawbridge", assets=ASSETS)

    concrete_matte = model.material("concrete_matte", rgba=(0.69, 0.70, 0.72, 1.0))
    steel_satin = model.material("steel_satin", rgba=(0.39, 0.43, 0.46, 1.0))
    graphite_matte = model.material("graphite_matte", rgba=(0.15, 0.16, 0.17, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    seal_dark = model.material("seal_dark", rgba=(0.10, 0.11, 0.12, 1.0))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def x_section(
        x: float,
        *,
        y_center: float,
        width: float,
        z_bottom: float,
        z_top: float,
        chamfer: float,
    ) -> list[tuple[float, float, float]]:
        y_min = y_center - width * 0.5
        y_max = y_center + width * 0.5
        return [
            (x, y_min + chamfer, z_bottom),
            (x, y_max - chamfer, z_bottom),
            (x, y_max, z_bottom + chamfer),
            (x, y_max, z_top - chamfer),
            (x, y_max - chamfer, z_top),
            (x, y_min + chamfer, z_top),
            (x, y_min, z_top - chamfer),
            (x, y_min, z_bottom + chamfer),
        ]

    left_girder_mesh = save_mesh(
        "drawbridge_left_girder.obj",
        section_loft(
            [
                x_section(0.26, y_center=1.68, width=0.42, z_bottom=-0.18, z_top=0.36, chamfer=0.04),
                x_section(1.40, y_center=1.68, width=0.42, z_bottom=-0.14, z_top=0.36, chamfer=0.04),
                x_section(4.90, y_center=1.67, width=0.38, z_bottom=-0.08, z_top=0.32, chamfer=0.035),
                x_section(8.55, y_center=1.65, width=0.32, z_bottom=-0.02, z_top=0.27, chamfer=0.03),
            ]
        ),
    )
    right_girder_mesh = save_mesh(
        "drawbridge_right_girder.obj",
        section_loft(
            [
                x_section(0.26, y_center=-1.68, width=0.42, z_bottom=-0.18, z_top=0.36, chamfer=0.04),
                x_section(1.40, y_center=-1.68, width=0.42, z_bottom=-0.14, z_top=0.36, chamfer=0.04),
                x_section(4.90, y_center=-1.67, width=0.38, z_bottom=-0.08, z_top=0.32, chamfer=0.035),
                x_section(8.55, y_center=-1.65, width=0.32, z_bottom=-0.02, z_top=0.27, chamfer=0.03),
            ]
        ),
    )

    abutment_frame = model.part("abutment_frame")
    abutment_frame.inertial = Inertial.from_geometry(
        Box((3.95, 6.20, 3.15)),
        mass=152000.0,
        origin=Origin(xyz=(-1.20, 0.0, 1.58)),
    )
    abutment_frame.visual(
        Box((3.80, 6.20, 0.90)),
        origin=Origin(xyz=(-1.25, 0.0, 0.45)),
        material=concrete_matte,
        name="foundation_plinth",
    )
    abutment_frame.visual(
        Box((0.76, 5.70, 2.55)),
        origin=Origin(xyz=(-2.63, 0.0, 2.175)),
        material=concrete_matte,
        name="backwall",
    )
    abutment_frame.visual(
        Box((1.40, 4.80, 0.82)),
        origin=Origin(xyz=(-1.65, 0.0, 1.31)),
        material=steel_satin,
        name="approach_support",
    )
    abutment_frame.visual(
        Box((1.56, 2.44, 0.34)),
        origin=Origin(xyz=(-1.28, 0.0, 1.85)),
        material=steel_satin,
        name="approach_plate",
    )
    abutment_frame.visual(
        Box((1.60, 3.20, 0.04)),
        origin=Origin(xyz=(-1.23, 0.0, 2.03)),
        material=graphite_matte,
        name="running_surface",
    )
    abutment_frame.visual(
        Box((0.16, 2.72, 0.10)),
        origin=Origin(xyz=(-0.13, 0.0, 1.45)),
        material=machined_steel,
        name="seal_land",
    )
    abutment_frame.visual(
        Box((1.34, 0.18, 0.20)),
        origin=Origin(xyz=(-1.33, 1.69, 2.10)),
        material=steel_satin,
        name="left_approach_curb",
    )
    abutment_frame.visual(
        Box((1.34, 0.18, 0.20)),
        origin=Origin(xyz=(-1.33, -1.69, 2.10)),
        material=steel_satin,
        name="right_approach_curb",
    )
    abutment_frame.visual(
        Box((0.44, 3.56, 0.06)),
        origin=Origin(xyz=(-0.06, 0.0, 1.47)),
        material=machined_steel,
        name="sill_plate",
    )
    abutment_frame.visual(
        Box((0.56, 3.30, 0.60)),
        origin=Origin(xyz=(-0.22, 0.0, 1.20)),
        material=steel_satin,
        name="cross_tie",
    )
    abutment_frame.visual(
        Box((1.00, 0.46, 2.25)),
        origin=Origin(xyz=(-0.12, 2.61, 2.025)),
        material=steel_satin,
        name="left_tower",
    )
    abutment_frame.visual(
        Box((1.00, 0.46, 2.25)),
        origin=Origin(xyz=(-0.12, -2.61, 2.025)),
        material=steel_satin,
        name="right_tower",
    )
    abutment_frame.visual(
        Box((0.44, 0.24, 0.76)),
        origin=Origin(xyz=(0.04, 2.50, 1.55)),
        material=steel_satin,
        name="left_bearing_arm",
    )
    abutment_frame.visual(
        Box((0.44, 0.24, 0.76)),
        origin=Origin(xyz=(0.04, -2.50, 1.55)),
        material=steel_satin,
        name="right_bearing_arm",
    )
    abutment_frame.visual(
        Cylinder(radius=0.40, length=0.08),
        origin=Origin(xyz=(0.0, 2.34, 1.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_satin,
        name="left_bearing_housing",
    )
    abutment_frame.visual(
        Cylinder(radius=0.40, length=0.08),
        origin=Origin(xyz=(0.0, -2.34, 1.55), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_satin,
        name="right_bearing_housing",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        for bolt_index, (x_pos, z_pos) in enumerate(((-0.12, 1.73), (0.12, 1.37)), start=1):
            abutment_frame.visual(
                Cylinder(radius=0.03, length=0.04),
                origin=Origin(
                    xyz=(x_pos, side_sign * 2.40, z_pos),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=machined_steel,
                name=f"{side_name}_cap_bolt_{bolt_index}",
            )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((8.65, 4.10, 0.82)),
        mass=38500.0,
        origin=Origin(xyz=(4.25, 0.0, 0.22)),
    )
    bridge_leaf.visual(
        Box((1.20, 3.66, 0.36)),
        origin=Origin(xyz=(0.86, 0.0, 0.17)),
        material=steel_satin,
        name="heel_block",
    )
    bridge_leaf.visual(
        Box((0.34, 3.52, 0.06)),
        origin=Origin(xyz=(0.20, 0.0, -0.02)),
        material=machined_steel,
        name="heel_pad",
    )
    bridge_leaf.visual(
        Box((0.56, 0.18, 0.78)),
        origin=Origin(xyz=(0.18, 1.92, 0.07)),
        material=steel_satin,
        name="left_root_cheek",
    )
    bridge_leaf.visual(
        Box((0.56, 0.18, 0.78)),
        origin=Origin(xyz=(0.18, -1.92, 0.07)),
        material=steel_satin,
        name="right_root_cheek",
    )
    bridge_leaf.visual(left_girder_mesh, material=steel_satin, name="left_box_girder")
    bridge_leaf.visual(right_girder_mesh, material=steel_satin, name="right_box_girder")
    bridge_leaf.visual(
        Box((6.80, 3.42, 0.18)),
        origin=Origin(xyz=(4.75, 0.0, 0.39)),
        material=steel_satin,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((4.90, 0.56, 0.24)),
        origin=Origin(xyz=(4.15, 0.0, 0.21)),
        material=steel_satin,
        name="center_spine",
    )
    for rib_index, rib_x in enumerate((1.55, 3.20, 4.85, 6.50), start=1):
        bridge_leaf.visual(
            Box((0.18, 3.10, 0.24)),
            origin=Origin(xyz=(rib_x, 0.0, 0.21)),
            material=steel_satin,
            name=f"cross_rib_{rib_index}",
        )
    bridge_leaf.visual(
        Box((6.64, 3.20, 0.04)),
        origin=Origin(xyz=(4.73, 0.0, 0.48)),
        material=graphite_matte,
        name="running_surface",
    )
    bridge_leaf.visual(
        Box((6.56, 0.18, 0.18)),
        origin=Origin(xyz=(4.75, 1.77, 0.57)),
        material=steel_satin,
        name="left_curb",
    )
    bridge_leaf.visual(
        Box((6.56, 0.18, 0.18)),
        origin=Origin(xyz=(4.75, -1.77, 0.57)),
        material=steel_satin,
        name="right_curb",
    )
    bridge_leaf.visual(
        Box((0.22, 3.44, 0.06)),
        origin=Origin(xyz=(8.16, 0.0, 0.44)),
        material=machined_steel,
        name="nose_lip",
    )
    bridge_leaf.visual(
        Box((0.28, 2.68, 0.05)),
        origin=Origin(xyz=(0.20, 0.0, 0.27)),
        material=machined_steel,
        name="seal_retainer",
    )
    bridge_leaf.visual(
        Box((0.12, 2.64, 0.06)),
        origin=Origin(xyz=(0.08, 0.0, 0.27)),
        material=seal_dark,
        name="root_seal",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.28, length=0.18),
        origin=Origin(xyz=(0.0, 2.21, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.28, length=0.18),
        origin=Origin(xyz=(0.0, -2.21, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="right_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.36, length=0.12),
        origin=Origin(xyz=(0.0, 2.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_satin,
        name="left_trunnion_collar",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.36, length=0.12),
        origin=Origin(xyz=(0.0, -2.06, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_satin,
        name="right_trunnion_collar",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=abutment_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=850000.0, velocity=0.18, lower=0.0, upper=1.22),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    abutment_frame = object_model.get_part("abutment_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    leaf_hinge = object_model.get_articulation("leaf_hinge")

    frame_running_surface = abutment_frame.get_visual("running_surface")
    seal_land = abutment_frame.get_visual("seal_land")
    frame_approach_plate = abutment_frame.get_visual("approach_plate")
    sill_plate = abutment_frame.get_visual("sill_plate")
    left_bearing_housing = abutment_frame.get_visual("left_bearing_housing")
    right_bearing_housing = abutment_frame.get_visual("right_bearing_housing")

    leaf_running_surface = bridge_leaf.get_visual("running_surface")
    deck_plate = bridge_leaf.get_visual("deck_plate")
    heel_pad = bridge_leaf.get_visual("heel_pad")
    left_trunnion = bridge_leaf.get_visual("left_trunnion")
    right_trunnion = bridge_leaf.get_visual("right_trunnion")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    rest_leaf_aabb = ctx.part_world_aabb(bridge_leaf)
    rest_frame_aabb = ctx.part_world_aabb(abutment_frame)
    ctx.check(
        "leaf hinge axis is transverse to the deck",
        tuple(round(v, 6) for v in leaf_hinge.axis) == (0.0, -1.0, 0.0),
        f"unexpected hinge axis {leaf_hinge.axis}",
    )

    if rest_leaf_aabb is not None:
        leaf_span = rest_leaf_aabb[1][0] - rest_leaf_aabb[0][0]
        leaf_width = rest_leaf_aabb[1][1] - rest_leaf_aabb[0][1]
        leaf_depth = rest_leaf_aabb[1][2] - rest_leaf_aabb[0][2]
        ctx.check(
            "bridge leaf has realistic heavy-span proportions",
            leaf_span > 8.0 and 3.5 < leaf_width < 5.0 and leaf_depth > 0.65,
            (
                f"leaf span/width/depth are {leaf_span:.3f}, {leaf_width:.3f}, {leaf_depth:.3f}; "
                "expected a full-scale heavy single-leaf bridge"
            ),
        )
    if rest_frame_aabb is not None:
        ctx.check(
            "abutment frame clearly supports hinge line",
            rest_frame_aabb[1][2] > 3.0 and rest_frame_aabb[1][1] - rest_frame_aabb[0][1] > 5.5,
            f"frame top only reaches z={rest_frame_aabb[1][2]:.3f}",
        )

    with ctx.pose({leaf_hinge: 0.0}):
        ctx.expect_gap(
            bridge_leaf,
            abutment_frame,
            axis="z",
            positive_elem=heel_pad,
            negative_elem=sill_plate,
            max_gap=0.0005,
            max_penetration=0.0,
            name="heel pad seats cleanly on sill plate at rest",
        )
        ctx.expect_overlap(
            bridge_leaf,
            abutment_frame,
            axes="x",
            elem_a=heel_pad,
            elem_b=sill_plate,
            min_overlap=0.12,
            name="heel pad overlaps sill plate along bridge length",
        )
        ctx.expect_overlap(
            bridge_leaf,
            abutment_frame,
            axes="y",
            elem_a=heel_pad,
            elem_b=sill_plate,
            min_overlap=3.45,
            name="heel pad overlaps sill plate across bridge width",
        )
        ctx.expect_gap(
            bridge_leaf,
            abutment_frame,
            axis="x",
            positive_elem=bridge_leaf.get_visual("root_seal"),
            negative_elem=seal_land,
            min_gap=0.04,
            max_gap=0.08,
            name="root seal keeps the interface break tight but legible",
        )
        ctx.expect_overlap(
            bridge_leaf,
            abutment_frame,
            axes="y",
            elem_a=bridge_leaf.get_visual("root_seal"),
            elem_b=seal_land,
            min_overlap=2.6,
            name="root seal spans the main carriageway width across the seam",
        )
        ctx.expect_contact(
            bridge_leaf,
            abutment_frame,
            elem_a=left_trunnion,
            elem_b=left_bearing_housing,
            contact_tol=1e-6,
            name="left trunnion is visibly seated in its bearing",
        )
        ctx.expect_contact(
            bridge_leaf,
            abutment_frame,
            elem_a=right_trunnion,
            elem_b=right_bearing_housing,
            contact_tol=1e-6,
            name="right trunnion is visibly seated in its bearing",
        )
        ctx.expect_overlap(
            bridge_leaf,
            abutment_frame,
            axes="xz",
            elem_a=left_trunnion,
            elem_b=left_bearing_housing,
            min_overlap=0.56,
            name="left hinge bearing remains coaxially aligned",
        )
        ctx.expect_overlap(
            bridge_leaf,
            abutment_frame,
            axes="xz",
            elem_a=right_trunnion,
            elem_b=right_bearing_housing,
            min_overlap=0.56,
            name="right hinge bearing remains coaxially aligned",
        )
        frame_surface_aabb = ctx.part_element_world_aabb(abutment_frame, elem=frame_running_surface)
        leaf_surface_aabb = ctx.part_element_world_aabb(bridge_leaf, elem=leaf_running_surface)
        deck_aabb = ctx.part_element_world_aabb(bridge_leaf, elem=deck_plate)
        approach_aabb = ctx.part_element_world_aabb(abutment_frame, elem=frame_approach_plate)
        if frame_surface_aabb is not None and leaf_surface_aabb is not None:
            frame_top = frame_surface_aabb[1][2]
            leaf_top = leaf_surface_aabb[1][2]
            ctx.check(
                "traffic surface levels remain in the same finished elevation class",
                abs(frame_top - leaf_top) <= 0.02,
                f"frame top z={frame_top:.4f}, leaf top z={leaf_top:.4f}",
            )
        if deck_aabb is not None and approach_aabb is not None:
            ctx.check(
                "main bridge deck is intentionally set back from the hinge for open-pose clearance",
                deck_aabb[0][0] >= 1.30,
                f"deck leading edge starts at x={deck_aabb[0][0]:.3f}; expected >= 1.30",
            )

    with ctx.pose({leaf_hinge: 1.0}):
        ctx.expect_contact(
            bridge_leaf,
            abutment_frame,
            elem_a=left_trunnion,
            elem_b=left_bearing_housing,
            contact_tol=1e-6,
            name="left trunnion stays mounted through travel",
        )
        ctx.expect_contact(
            bridge_leaf,
            abutment_frame,
            elem_a=right_trunnion,
            elem_b=right_bearing_housing,
            contact_tol=1e-6,
            name="right trunnion stays mounted through travel",
        )
        ctx.expect_gap(
            bridge_leaf,
            abutment_frame,
            axis="z",
            positive_elem=heel_pad,
            negative_elem=sill_plate,
            min_gap=0.04,
            name="heel pad clears the sill once the bridge opens",
        )
        open_leaf_aabb = ctx.part_world_aabb(bridge_leaf)
        if rest_leaf_aabb is not None and open_leaf_aabb is not None:
            ctx.check(
                "bridge leaf lifts decisively when opened",
                open_leaf_aabb[1][2] > rest_leaf_aabb[1][2] + 6.0,
                (
                    f"open max z {open_leaf_aabb[1][2]:.3f} "
                    f"should exceed rest max z {rest_leaf_aabb[1][2]:.3f} by at least 6.0"
                ),
            )
            ctx.check(
                "bridge leaf tip swings back over the hinge as expected",
                open_leaf_aabb[1][0] < rest_leaf_aabb[1][0] - 3.2,
                (
                    f"open max x {open_leaf_aabb[1][0]:.3f} "
                    f"should retract from rest max x {rest_leaf_aabb[1][0]:.3f}"
                ),
            )

    limits = leaf_hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({leaf_hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="leaf_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="leaf_hinge_lower_no_floating")
        with ctx.pose({leaf_hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="leaf_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="leaf_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
