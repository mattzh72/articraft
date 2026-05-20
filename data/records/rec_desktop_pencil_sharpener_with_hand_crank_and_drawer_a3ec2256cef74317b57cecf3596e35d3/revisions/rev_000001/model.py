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
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rounded_section(width: float, depth: float, radius: float, z: float):
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="schoolhouse_pencil_sharpener")

    cast_green = Material("hammered_sage_cast_metal", rgba=(0.34, 0.43, 0.36, 1.0))
    dark = Material("dark_opening_shadow", rgba=(0.015, 0.014, 0.012, 1.0))
    drawer_mat = Material("dark_shavings_drawer", rgba=(0.09, 0.10, 0.10, 1.0))
    nickel = Material("brushed_nickel", rgba=(0.72, 0.70, 0.65, 1.0))
    steel = Material("dull_steel", rgba=(0.52, 0.54, 0.53, 1.0))
    black = Material("black_plastic", rgba=(0.02, 0.02, 0.018, 1.0))
    wood = Material("worn_wood_handle", rgba=(0.58, 0.36, 0.18, 1.0))

    body = model.part("body")

    # Rounded, slightly tapering cast-metal "schoolhouse" tower.
    housing = section_loft(
        [
            _rounded_section(0.112, 0.116, 0.013, 0.050),
            _rounded_section(0.109, 0.112, 0.015, 0.100),
            _rounded_section(0.093, 0.096, 0.020, 0.157),
            _rounded_section(0.066, 0.070, 0.020, 0.188),
        ]
    )
    body.visual(
        mesh_from_geometry(housing, "tapered_cast_body"),
        material=cast_green,
        name="tapered_cast_body",
    )

    # Drawer sleeve and pedestal frame.  The center is open so the tray can slide.
    body.visual(
        Box((0.116, 0.118, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cast_green,
        name="base_floor",
    )
    body.visual(
        Box((0.018, 0.118, 0.054)),
        origin=Origin(xyz=(-0.049, 0.0, 0.032)),
        material=cast_green,
        name="side_cheek_0",
    )
    body.visual(
        Box((0.018, 0.118, 0.054)),
        origin=Origin(xyz=(0.049, 0.0, 0.032)),
        material=cast_green,
        name="side_cheek_1",
    )
    body.visual(
        Box((0.116, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.049, 0.054)),
        material=cast_green,
        name="front_lintel",
    )
    body.visual(
        Box((0.116, 0.010, 0.036)),
        origin=Origin(xyz=(0.0, 0.054, 0.034)),
        material=cast_green,
        name="rear_wall",
    )

    # Front entry port, bevel ring and visible dark throat underneath the flap.
    body.visual(
        Cylinder(radius=0.017, length=0.003),
        origin=Origin(xyz=(0.0, -0.058, 0.113), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="entry_shadow",
    )
    entry_ring = TorusGeometry(radius=0.019, tube=0.0022, radial_segments=24, tubular_segments=12)
    body.visual(
        mesh_from_geometry(entry_ring, "entry_bezel"),
        origin=Origin(xyz=(0.0, -0.0565, 0.113), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=nickel,
        name="entry_bezel",
    )

    # Side crank support bushing and small rivets/screw heads for classroom-sharpener character.
    side_bushing = TorusGeometry(radius=0.008, tube=0.005, radial_segments=24, tubular_segments=12)
    body.visual(
        mesh_from_geometry(side_bushing, "side_bushing"),
        origin=Origin(xyz=(0.0555, 0.000, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nickel,
        name="side_bushing",
    )
    for i, (x, z) in enumerate(((-0.038, 0.032), (0.038, 0.032), (-0.032, 0.168), (0.032, 0.168))):
        screw_y = -0.0575 if z < 0.060 else -0.045
        body.visual(
            Cylinder(radius=0.004, length=0.0025),
            origin=Origin(xyz=(x, screw_y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=nickel,
            name=f"front_screw_{i}",
        )

    # Exposed hinge knuckles fixed to the body, split around the moving flap knuckle.
    body.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(-0.019, -0.061, 0.138), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="flap_hinge_knuckle_0",
    )
    body.visual(
        Box((0.018, 0.006, 0.010)),
        origin=Origin(xyz=(-0.019, -0.056, 0.132)),
        material=steel,
        name="flap_hinge_leaf_0",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.019, -0.061, 0.138), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="flap_hinge_knuckle_1",
    )
    body.visual(
        Box((0.018, 0.006, 0.010)),
        origin=Origin(xyz=(0.019, -0.056, 0.132)),
        material=steel,
        name="flap_hinge_leaf_1",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.082, 0.009, 0.043)),
        origin=Origin(xyz=(0.0, -0.0045, 0.0)),
        material=drawer_mat,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.066, 0.106, 0.006)),
        origin=Origin(xyz=(0.0, 0.050, -0.0145)),
        material=drawer_mat,
        name="tray_bottom",
    )
    drawer.visual(
        Box((0.005, 0.098, 0.025)),
        origin=Origin(xyz=(-0.033, 0.052, -0.003)),
        material=drawer_mat,
        name="tray_wall_0",
    )
    drawer.visual(
        Box((0.005, 0.098, 0.025)),
        origin=Origin(xyz=(0.033, 0.052, -0.003)),
        material=drawer_mat,
        name="tray_wall_1",
    )
    drawer.visual(
        Box((0.066, 0.006, 0.025)),
        origin=Origin(xyz=(0.0, 0.103, -0.003)),
        material=drawer_mat,
        name="tray_back",
    )
    drawer.visual(
        Cylinder(radius=0.007, length=0.005),
        origin=Origin(xyz=(0.0, -0.010, 0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="drawer_pull",
    )
    drawer_slide = model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.058, 0.031)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.060),
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.005, length=0.040),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="side_shaft",
    )
    crank.visual(
        Cylinder(radius=0.016, length=0.010),
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nickel,
        name="crank_hub",
    )
    crank.visual(
        Box((0.010, 0.008, 0.072)),
        origin=Origin(xyz=(0.047, 0.0, -0.036)),
        material=nickel,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.058, 0.0, -0.071), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="handle_pin",
    )
    crank.visual(
        Cylinder(radius=0.0085, length=0.026),
        origin=Origin(xyz=(0.073, 0.0, -0.071), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wood,
        name="wood_grip",
    )
    crank_spin = model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.057, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    dust_flap = model.part("dust_flap")
    dust_flap.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="flap_knuckle",
    )
    dust_flap.visual(
        Box((0.052, 0.003, 0.044)),
        origin=Origin(xyz=(0.0, -0.0015, -0.024)),
        material=nickel,
        name="flap_plate",
    )
    dust_flap.visual(
        Box((0.020, 0.0035, 0.006)),
        origin=Origin(xyz=(0.0, -0.003, -0.046)),
        material=nickel,
        name="finger_lip",
    )
    flap_hinge = model.articulation(
        "body_to_dust_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dust_flap,
        origin=Origin(xyz=(0.0, -0.061, 0.138)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.5, lower=0.0, upper=1.25),
    )

    # Mark important joints in metadata for reviewers without changing behavior.
    drawer_slide.meta["mechanism"] = "pullout waste drawer"
    crank_spin.meta["mechanism"] = "continuous side crank"
    flap_hinge.meta["mechanism"] = "hinged dust flap"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    dust_flap = object_model.get_part("dust_flap")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    crank_joint = object_model.get_articulation("body_to_crank")
    flap_joint = object_model.get_articulation("body_to_dust_flap")

    ctx.allow_overlap(
        body,
        crank,
        elem_a="side_bushing",
        elem_b="side_shaft",
        reason="The crank shaft is intentionally captured by the side bushing bearing.",
    )

    ctx.check(
        "primary mechanisms are articulated",
        drawer_joint.articulation_type == ArticulationType.PRISMATIC
        and crank_joint.articulation_type == ArticulationType.CONTINUOUS
        and flap_joint.articulation_type == ArticulationType.REVOLUTE,
        details="Expected prismatic drawer, continuous crank, and revolute dust flap.",
    )

    ctx.expect_overlap(
        dust_flap,
        body,
        axes="xz",
        elem_a="flap_plate",
        elem_b="entry_shadow",
        min_overlap=0.026,
        name="closed flap covers pencil entry port",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="x",
        inner_elem="tray_bottom",
        outer_elem="base_floor",
        margin=0.0,
        name="drawer tray runs inside body cheeks",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="y",
        elem_a="tray_bottom",
        elem_b="base_floor",
        min_overlap=0.095,
        name="closed drawer is deeply inserted",
    )
    ctx.expect_overlap(
        crank,
        body,
        axes="yz",
        elem_a="side_shaft",
        elem_b="side_bushing",
        min_overlap=0.009,
        name="crank shaft is centered in side bushing",
    )

    rest_drawer = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.060}):
        extended_drawer = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="tray_bottom",
            elem_b="base_floor",
            min_overlap=0.035,
            name="extended drawer keeps retained insertion",
        )
    ctx.check(
        "drawer pulls toward the front",
        rest_drawer is not None
        and extended_drawer is not None
        and extended_drawer[1] < rest_drawer[1] - 0.055,
        details=f"rest={rest_drawer}, extended={extended_drawer}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(dust_flap, elem="flap_plate")
    with ctx.pose({flap_joint: 1.10}):
        open_flap_aabb = ctx.part_element_world_aabb(dust_flap, elem="flap_plate")
    closed_center_y = (closed_flap_aabb[0][1] + closed_flap_aabb[1][1]) / 2.0 if closed_flap_aabb else None
    open_center_y = (open_flap_aabb[0][1] + open_flap_aabb[1][1]) / 2.0 if open_flap_aabb else None
    closed_center_z = (closed_flap_aabb[0][2] + closed_flap_aabb[1][2]) / 2.0 if closed_flap_aabb else None
    open_center_z = (open_flap_aabb[0][2] + open_flap_aabb[1][2]) / 2.0 if open_flap_aabb else None
    ctx.check(
        "dust flap swings outward and upward",
        closed_center_y is not None
        and open_center_y is not None
        and closed_center_z is not None
        and open_center_z is not None
        and open_center_y < closed_center_y - 0.012
        and open_center_z > closed_center_z + 0.010,
        details=(
            f"closed_center=(y={closed_center_y}, z={closed_center_z}), "
            f"open_center=(y={open_center_y}, z={open_center_z})"
        ),
    )

    return ctx.report()


object_model = build_object_model()
