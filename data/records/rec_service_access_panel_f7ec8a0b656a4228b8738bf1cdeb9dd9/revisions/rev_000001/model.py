from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _x_cylinder_origin(x: float, y: float, z: float) -> Origin:
    """Origin for a cylinder whose axis is the object +X direction."""
    return Origin(xyz=(x, y, z), rpy=(0.0, pi / 2.0, 0.0))


def _rounded_panel_mesh(width_y: float, height_z: float, thickness_x: float, radius: float, name: str):
    """Build a rounded rectangular plate whose thickness is along local +X."""
    # ExtrudeWithHolesGeometry works in local XY and extrudes along local Z.
    # Rotating about Y maps that extrusion to Articraft/world +X; the profile's
    # local Y remains the service-panel horizontal span and local X becomes Z.
    profile = rounded_rect_profile(height_z, width_y, radius, corner_segments=8)
    geom = ExtrudeWithHolesGeometry(profile, (), thickness_x, center=True)
    geom.rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, name)


def _access_frame_mesh(
    outer_y: float,
    outer_z: float,
    opening_y: float,
    opening_z: float,
    thickness_x: float,
):
    outer = rounded_rect_profile(outer_z, outer_y, 0.040, corner_segments=10)
    inner = rounded_rect_profile(opening_z, opening_y, 0.018, corner_segments=8)
    geom = ExtrudeWithHolesGeometry(outer, (inner,), thickness_x, center=True)
    geom.rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, "enclosure_face")


def _add_screw(part, name: str, x: float, y: float, z: float, material, slot_material) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=_x_cylinder_origin(x, y, z),
        material=material,
        name=f"{name}_head",
    )
    part.visual(
        Box((0.0025, 0.015, 0.0025)),
        origin=Origin(xyz=(x + 0.0045, y, z)),
        material=slot_material,
        name=f"{name}_slot",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_service_access_panel")

    powder_grey = model.material("textured_powder_coat", rgba=(0.18, 0.20, 0.20, 1.0))
    inner_shadow = model.material("dark_recess", rgba=(0.025, 0.028, 0.028, 1.0))
    safety_panel = model.material("molded_olive_panel", rgba=(0.24, 0.31, 0.22, 1.0))
    worn_edge = model.material("edge_wear", rgba=(0.56, 0.58, 0.54, 1.0))
    hinge_steel = model.material("black_oxide_steel", rgba=(0.035, 0.035, 0.035, 1.0))
    zinc = model.material("zinc_fastener", rgba=(0.62, 0.64, 0.60, 1.0))
    rubber = model.material("compressed_black_gasket", rgba=(0.006, 0.006, 0.005, 1.0))

    # Root: a single heavy enclosure face with a real through-opening and a
    # raised service-frame land.  Coordinate convention: +X is outward from the
    # utility enclosure, +Y is across the panel, +Z is vertical.
    enclosure = model.part("enclosure")
    enclosure.visual(
        _access_frame_mesh(
            outer_y=0.750,
            outer_z=0.520,
            opening_y=0.520,
            opening_z=0.340,
            thickness_x=0.040,
        ),
        material=powder_grey,
        name="enclosure_face",
    )

    # Dark inner return surfaces make the cut-through opening read as a thick,
    # serviceable enclosure rather than a painted rectangle.
    enclosure.visual(
        Box((0.070, 0.014, 0.340)),
        origin=Origin(xyz=(-0.018, -0.267, 0.0)),
        material=inner_shadow,
        name="left_return",
    )
    enclosure.visual(
        Box((0.070, 0.014, 0.340)),
        origin=Origin(xyz=(-0.018, 0.267, 0.0)),
        material=inner_shadow,
        name="right_return",
    )
    enclosure.visual(
        Box((0.070, 0.520, 0.014)),
        origin=Origin(xyz=(-0.018, 0.0, 0.177)),
        material=inner_shadow,
        name="top_return",
    )
    enclosure.visual(
        Box((0.070, 0.520, 0.014)),
        origin=Origin(xyz=(-0.018, 0.0, -0.177)),
        material=inner_shadow,
        name="bottom_return",
    )

    # Raised, thicker landing frame around the service opening.
    enclosure.visual(
        Box((0.024, 0.615, 0.044)),
        origin=Origin(xyz=(0.032, 0.0, 0.192)),
        material=powder_grey,
        name="top_frame_land",
    )
    enclosure.visual(
        Box((0.024, 0.615, 0.044)),
        origin=Origin(xyz=(0.032, 0.0, -0.192)),
        material=powder_grey,
        name="bottom_frame_land",
    )
    enclosure.visual(
        Box((0.024, 0.044, 0.340)),
        origin=Origin(xyz=(0.032, -0.282, 0.0)),
        material=powder_grey,
        name="hinge_frame_land",
    )
    enclosure.visual(
        Box((0.024, 0.044, 0.340)),
        origin=Origin(xyz=(0.032, 0.282, 0.0)),
        material=powder_grey,
        name="latch_frame_land",
    )

    # Hinge-side reinforcing pad and fixed hinge leaf.
    enclosure.visual(
        Box((0.025, 0.098, 0.450)),
        origin=Origin(xyz=(0.0325, -0.318, 0.0)),
        material=powder_grey,
        name="hinge_mount_pad",
    )
    enclosure.visual(
        Box((0.012, 0.070, 0.430)),
        origin=Origin(xyz=(0.041, -0.340, 0.0)),
        material=hinge_steel,
        name="fixed_hinge_leaf",
    )
    for idx, z in enumerate((-0.160, 0.0, 0.160)):
        enclosure.visual(
            Cylinder(radius=0.018, length=0.085),
            origin=Origin(xyz=(0.065, -0.310, z)),
            material=hinge_steel,
            name=f"fixed_knuckle_{idx}",
        )

    # Latch striker and serviceable fastening points on the rigid frame.
    enclosure.visual(
        Box((0.025, 0.098, 0.300)),
        origin=Origin(xyz=(0.0325, 0.318, 0.0)),
        material=powder_grey,
        name="latch_mount_pad",
    )
    enclosure.visual(
        Box((0.010, 0.050, 0.115)),
        origin=Origin(xyz=(0.049, 0.326, 0.0)),
        material=hinge_steel,
        name="strike_plate",
    )
    enclosure.visual(
        Box((0.004, 0.016, 0.052)),
        origin=Origin(xyz=(0.056, 0.302, 0.0)),
        material=inner_shadow,
        name="strike_slot",
    )
    for idx, (y, z) in enumerate(
        (
            (-0.300, 0.214),
            (0.300, 0.214),
            (-0.300, -0.214),
            (0.300, -0.214),
            (-0.338, 0.120),
            (-0.338, -0.120),
            (0.338, 0.120),
            (0.338, -0.120),
        )
    ):
        _add_screw(enclosure, f"frame_screw_{idx}", 0.048, y, z, zinc, inner_shadow)

    panel = model.part("panel")
    panel.visual(
        _rounded_panel_mesh(0.590, 0.410, 0.030, 0.026, "panel_slab"),
        origin=Origin(xyz=(0.0, 0.315, 0.0)),
        material=safety_panel,
        name="panel_slab",
    )
    # Pressed/ribbed door edges, a replaceable seal, and scuffed metal cues.
    panel.visual(
        Box((0.014, 0.540, 0.036)),
        origin=Origin(xyz=(0.022, 0.300, 0.168)),
        material=safety_panel,
        name="top_panel_rib",
    )
    panel.visual(
        Box((0.014, 0.540, 0.036)),
        origin=Origin(xyz=(0.022, 0.300, -0.168)),
        material=safety_panel,
        name="bottom_panel_rib",
    )
    panel.visual(
        Box((0.014, 0.036, 0.330)),
        origin=Origin(xyz=(0.022, 0.055, 0.0)),
        material=safety_panel,
        name="hinge_panel_rib",
    )
    panel.visual(
        Box((0.014, 0.036, 0.330)),
        origin=Origin(xyz=(0.022, 0.535, 0.0)),
        material=safety_panel,
        name="latch_panel_rib",
    )
    panel.visual(
        Box((0.006, 0.485, 0.020)),
        origin=Origin(xyz=(-0.015, 0.300, 0.145)),
        material=rubber,
        name="top_gasket",
    )
    panel.visual(
        Box((0.006, 0.485, 0.020)),
        origin=Origin(xyz=(-0.015, 0.300, -0.145)),
        material=rubber,
        name="bottom_gasket",
    )
    panel.visual(
        Box((0.006, 0.020, 0.300)),
        origin=Origin(xyz=(-0.015, 0.065, 0.0)),
        material=rubber,
        name="hinge_gasket",
    )
    panel.visual(
        Box((0.006, 0.020, 0.300)),
        origin=Origin(xyz=(-0.015, 0.535, 0.0)),
        material=rubber,
        name="latch_gasket",
    )
    panel.visual(
        Box((0.010, 0.100, 0.100)),
        origin=Origin(xyz=(0.020, 0.505, 0.0)),
        material=safety_panel,
        name="latch_boss",
    )
    panel.visual(
        Box((0.003, 0.540, 0.010)),
        origin=Origin(xyz=(0.031, 0.300, 0.203)),
        material=worn_edge,
        name="top_worn_edge",
    )
    panel.visual(
        Box((0.003, 0.540, 0.010)),
        origin=Origin(xyz=(0.031, 0.300, -0.203)),
        material=worn_edge,
        name="bottom_worn_edge",
    )
    panel.visual(
        Box((0.012, 0.070, 0.410)),
        origin=Origin(xyz=(0.024, 0.032, 0.0)),
        material=hinge_steel,
        name="moving_hinge_leaf",
    )
    for idx, z in enumerate((-0.080, 0.080)):
        panel.visual(
            Cylinder(radius=0.018, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=hinge_steel,
            name=f"moving_knuckle_{idx}",
        )
    for idx, (y, z) in enumerate(
        (
            (0.090, 0.150),
            (0.300, 0.150),
            (0.510, 0.150),
            (0.090, -0.150),
            (0.300, -0.150),
            (0.510, -0.150),
            (0.535, 0.080),
            (0.535, -0.080),
        )
    ):
        _add_screw(panel, f"panel_screw_{idx}", 0.033, y, z, zinc, inner_shadow)

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=_x_cylinder_origin(0.0, 0.0, 0.0),
        material=hinge_steel,
        name="latch_dial",
    )
    latch.visual(
        Box((0.012, 0.096, 0.018)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=hinge_steel,
        name="turn_wing",
    )
    latch.visual(
        Box((0.003, 0.055, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=inner_shadow,
        name="driver_slot",
    )

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=panel,
        origin=Origin(xyz=(0.065, -0.310, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=65.0, velocity=1.2, lower=0.0, upper=1.65),
        motion_properties=MotionProperties(damping=0.15, friction=0.08),
    )
    model.articulation(
        "latch_turn",
        ArticulationType.REVOLUTE,
        parent=panel,
        child=latch,
        origin=Origin(xyz=(0.031, 0.505, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=pi / 2.0),
        motion_properties=MotionProperties(damping=0.04, friction=0.02),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    enclosure = object_model.get_part("enclosure")
    panel = object_model.get_part("panel")
    latch = object_model.get_part("latch")
    hinge = object_model.get_articulation("panel_hinge")
    latch_turn = object_model.get_articulation("latch_turn")

    with ctx.pose({hinge: 0.0, latch_turn: 0.0}):
        ctx.expect_gap(
            panel,
            enclosure,
            axis="x",
            min_gap=0.003,
            max_gap=0.010,
            positive_elem="bottom_gasket",
            negative_elem="bottom_frame_land",
            name="closed gasket stands just proud of frame",
        )
        ctx.expect_overlap(
            panel,
            enclosure,
            axes="yz",
            elem_a="panel_slab",
            elem_b="enclosure_face",
            min_overlap=0.34,
            name="door covers the service opening footprint",
        )
        ctx.expect_contact(
            latch,
            panel,
            elem_a="latch_dial",
            elem_b="latch_boss",
            contact_tol=0.0005,
            name="quarter turn latch seats on reinforced boss",
        )

    closed_aabb = ctx.part_element_world_aabb(panel, elem="panel_slab")
    with ctx.pose({hinge: 1.20}):
        opened_aabb = ctx.part_element_world_aabb(panel, elem="panel_slab")
        ctx.expect_gap(
            panel,
            enclosure,
            axis="x",
            min_gap=0.010,
            positive_elem="panel_slab",
            negative_elem="latch_frame_land",
            name="opened panel swings outward from frame",
        )

    ctx.check(
        "hinge carries free edge outward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][0] > closed_aabb[1][0] + 0.10,
        details=f"closed_aabb={closed_aabb}, opened_aabb={opened_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
