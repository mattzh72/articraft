from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)


OUTER_RING_X = 0.038
OUTER_RING_Y = 0.030
INNER_RING_X = 0.020
INNER_RING_Y = 0.016
RING_THICKNESS = 0.0048
RING_TOP_Z = RING_THICKNESS * 0.5

HINGE_RADIUS = 0.0010
HINGE_Z = RING_TOP_Z + HINGE_RADIUS
HINGE_BARREL_LENGTH = 0.0046
BODY_LUG_LENGTH = 0.0036
BODY_LUG_CENTER_OFFSET = (HINGE_BARREL_LENGTH + BODY_LUG_LENGTH) * 0.5

HINGE_PAD_HEIGHT = 0.0018
HINGE_PAD_BOTTOM_Z = RING_TOP_Z - 0.0003
HINGE_PAD_CENTER_Z = HINGE_PAD_BOTTOM_Z + (HINGE_PAD_HEIGHT * 0.5)
HINGE_PAD_DEPTH = 0.0022

TAB_LENGTH = 0.0050
TAB_WIDTH = 0.0064
TAB_THICKNESS = 0.0018
TAB_OVERLAP_WITH_BARREL = 0.0002
TAB_CENTER_OFFSET = HINGE_RADIUS + (TAB_LENGTH * 0.5) - TAB_OVERLAP_WITH_BARREL
TAB_CENTER_Z_LOCAL = (RING_TOP_Z + (TAB_THICKNESS * 0.5)) - HINGE_Z

GRIP_RIB_THICKNESS = 0.0006
GRIP_RIB_DEPTH = 0.0008
GRIP_RIB_CENTER_Z_LOCAL = TAB_CENTER_Z_LOCAL + (TAB_THICKNESS * 0.5) - 0.00012 + (
    GRIP_RIB_THICKNESS * 0.5
)
GRIP_RIB_OFFSET = TAB_CENTER_OFFSET + (TAB_LENGTH * 0.5) - (GRIP_RIB_DEPTH * 0.65)

NORTH_SOUTH_HINGE_Y = 0.0092
EAST_WEST_HINGE_X = 0.0112
OPEN_ANGLE = 1.0


def _ring_shell_mesh():
    outer_profile = superellipse_profile(OUTER_RING_X, OUTER_RING_Y, exponent=2.35, segments=72)
    inner_profile = superellipse_profile(INNER_RING_X, INNER_RING_Y, exponent=2.45, segments=64)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            [inner_profile],
            RING_THICKNESS,
            center=True,
        ),
        "flipper_ring_shell",
    )


def _tab_plate_mesh(length_along: str):
    if length_along == "y":
        profile = rounded_rect_profile(TAB_WIDTH, TAB_LENGTH, radius=0.00115, corner_segments=6)
        name = "flipper_tab_plate_y"
    else:
        profile = rounded_rect_profile(TAB_LENGTH, TAB_WIDTH, radius=0.00115, corner_segments=6)
        name = "flipper_tab_plate_x"
    return mesh_from_geometry(
        ExtrudeGeometry(profile, TAB_THICKNESS, center=True),
        name,
    )


def _add_ring_hinge_mount(ring_body, prefix: str, *, axis: str, hinge_coord: float, material) -> None:
    if axis == "x":
        pad_size = (BODY_LUG_LENGTH + 0.0006, HINGE_PAD_DEPTH, HINGE_PAD_HEIGHT)
        lug_rpy = (0.0, math.pi / 2.0, 0.0)
        for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
            x_pos = x_sign * BODY_LUG_CENTER_OFFSET
            ring_body.visual(
                Box(pad_size),
                origin=Origin(xyz=(x_pos, hinge_coord, HINGE_PAD_CENTER_Z)),
                material=material,
                name=f"{prefix}_{side_name}_pad",
            )
            ring_body.visual(
                Cylinder(radius=HINGE_RADIUS, length=BODY_LUG_LENGTH),
                origin=Origin(
                    xyz=(x_pos, hinge_coord, HINGE_Z),
                    rpy=lug_rpy,
                ),
                material=material,
                name=f"{prefix}_{side_name}_lug",
            )
    else:
        pad_size = (HINGE_PAD_DEPTH, BODY_LUG_LENGTH + 0.0006, HINGE_PAD_HEIGHT)
        lug_rpy = (math.pi / 2.0, 0.0, 0.0)
        for side_name, y_sign in (("lower", -1.0), ("upper", 1.0)):
            y_pos = y_sign * BODY_LUG_CENTER_OFFSET
            ring_body.visual(
                Box(pad_size),
                origin=Origin(xyz=(hinge_coord, y_pos, HINGE_PAD_CENTER_Z)),
                material=material,
                name=f"{prefix}_{side_name}_pad",
            )
            ring_body.visual(
                Cylinder(radius=HINGE_RADIUS, length=BODY_LUG_LENGTH),
                origin=Origin(
                    xyz=(hinge_coord, y_pos, HINGE_Z),
                    rpy=lug_rpy,
                ),
                material=material,
                name=f"{prefix}_{side_name}_lug",
            )


def _build_flipper_tab(model: ArticulatedObject, name: str, *, extend_axis: str, material, mesh_x, mesh_y):
    tab = model.part(name)

    if extend_axis in {"+y", "-y"}:
        sign = 1.0 if extend_axis == "+y" else -1.0
        tab.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name="barrel",
        )
        tab.visual(
            mesh_y,
            origin=Origin(xyz=(0.0, sign * TAB_CENTER_OFFSET, TAB_CENTER_Z_LOCAL)),
            material=material,
            name="paddle",
        )
        tab.visual(
            Box((TAB_WIDTH * 0.72, GRIP_RIB_DEPTH, GRIP_RIB_THICKNESS)),
            origin=Origin(
                xyz=(0.0, sign * GRIP_RIB_OFFSET, GRIP_RIB_CENTER_Z_LOCAL),
            ),
            material=material,
            name="grip_rib",
        )
        inertial_origin = Origin(xyz=(0.0, sign * 0.0026, TAB_CENTER_Z_LOCAL + 0.0003))
        inertial_box = Box((TAB_WIDTH, HINGE_BARREL_LENGTH + TAB_LENGTH, 0.0028))
    else:
        sign = 1.0 if extend_axis == "+x" else -1.0
        tab.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_BARREL_LENGTH),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=material,
            name="barrel",
        )
        tab.visual(
            mesh_x,
            origin=Origin(xyz=(sign * TAB_CENTER_OFFSET, 0.0, TAB_CENTER_Z_LOCAL)),
            material=material,
            name="paddle",
        )
        tab.visual(
            Box((GRIP_RIB_DEPTH, TAB_WIDTH * 0.72, GRIP_RIB_THICKNESS)),
            origin=Origin(
                xyz=(sign * GRIP_RIB_OFFSET, 0.0, GRIP_RIB_CENTER_Z_LOCAL),
            ),
            material=material,
            name="grip_rib",
        )
        inertial_origin = Origin(xyz=(sign * 0.0026, 0.0, TAB_CENTER_Z_LOCAL + 0.0003))
        inertial_box = Box((HINGE_BARREL_LENGTH + TAB_LENGTH, TAB_WIDTH, 0.0028))

    tab.inertial = Inertial.from_geometry(
        inertial_box,
        mass=0.0018,
        origin=inertial_origin,
    )
    return tab


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flipper_ring_fidget")

    body_metal = model.material("body_metal", rgba=(0.19, 0.21, 0.24, 1.0))
    tab_metal = model.material("tab_metal", rgba=(0.77, 0.79, 0.81, 1.0))

    ring_body = model.part("ring_body")
    ring_body.visual(_ring_shell_mesh(), material=body_metal, name="ring_shell")
    ring_body.inertial = Inertial.from_geometry(
        Box((OUTER_RING_X, OUTER_RING_Y, 0.0070)),
        mass=0.014,
        origin=Origin(xyz=(0.0, 0.0, 0.0011)),
    )

    _add_ring_hinge_mount(
        ring_body,
        "north_hinge",
        axis="x",
        hinge_coord=NORTH_SOUTH_HINGE_Y,
        material=body_metal,
    )
    _add_ring_hinge_mount(
        ring_body,
        "south_hinge",
        axis="x",
        hinge_coord=-NORTH_SOUTH_HINGE_Y,
        material=body_metal,
    )
    _add_ring_hinge_mount(
        ring_body,
        "east_hinge",
        axis="y",
        hinge_coord=EAST_WEST_HINGE_X,
        material=body_metal,
    )
    _add_ring_hinge_mount(
        ring_body,
        "west_hinge",
        axis="y",
        hinge_coord=-EAST_WEST_HINGE_X,
        material=body_metal,
    )

    tab_mesh_x = _tab_plate_mesh("x")
    tab_mesh_y = _tab_plate_mesh("y")

    north_tab = _build_flipper_tab(
        model,
        "north_tab",
        extend_axis="+y",
        material=tab_metal,
        mesh_x=tab_mesh_x,
        mesh_y=tab_mesh_y,
    )
    south_tab = _build_flipper_tab(
        model,
        "south_tab",
        extend_axis="-y",
        material=tab_metal,
        mesh_x=tab_mesh_x,
        mesh_y=tab_mesh_y,
    )
    east_tab = _build_flipper_tab(
        model,
        "east_tab",
        extend_axis="+x",
        material=tab_metal,
        mesh_x=tab_mesh_x,
        mesh_y=tab_mesh_y,
    )
    west_tab = _build_flipper_tab(
        model,
        "west_tab",
        extend_axis="-x",
        material=tab_metal,
        mesh_x=tab_mesh_x,
        mesh_y=tab_mesh_y,
    )

    model.articulation(
        "ring_to_north_tab",
        ArticulationType.REVOLUTE,
        parent=ring_body,
        child=north_tab,
        origin=Origin(xyz=(0.0, NORTH_SOUTH_HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=10.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "ring_to_south_tab",
        ArticulationType.REVOLUTE,
        parent=ring_body,
        child=south_tab,
        origin=Origin(xyz=(0.0, -NORTH_SOUTH_HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=10.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "ring_to_east_tab",
        ArticulationType.REVOLUTE,
        parent=ring_body,
        child=east_tab,
        origin=Origin(xyz=(EAST_WEST_HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=10.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "ring_to_west_tab",
        ArticulationType.REVOLUTE,
        parent=ring_body,
        child=west_tab,
        origin=Origin(xyz=(-EAST_WEST_HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=10.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ring_body = object_model.get_part("ring_body")
    north_tab = object_model.get_part("north_tab")
    south_tab = object_model.get_part("south_tab")
    east_tab = object_model.get_part("east_tab")
    west_tab = object_model.get_part("west_tab")
    north_joint = object_model.get_articulation("ring_to_north_tab")
    south_joint = object_model.get_articulation("ring_to_south_tab")
    east_joint = object_model.get_articulation("ring_to_east_tab")
    west_joint = object_model.get_articulation("ring_to_west_tab")

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

    for tab in (north_tab, south_tab, east_tab, west_tab):
        ctx.expect_contact(tab, ring_body, name=f"{tab.name} stays mounted to the ring body")
        ctx.expect_gap(
            tab,
            ring_body,
            axis="z",
            positive_elem="paddle",
            negative_elem="ring_shell",
            max_gap=0.0002,
            max_penetration=0.0,
            name=f"{tab.name} sits flush on the ring face when closed",
        )

    ctx.expect_origin_distance(
        north_tab,
        south_tab,
        axes="y",
        min_dist=0.0180,
        max_dist=0.0186,
        name="north and south tabs are symmetrically spaced",
    )
    ctx.expect_origin_distance(
        east_tab,
        west_tab,
        axes="x",
        min_dist=0.0220,
        max_dist=0.0226,
        name="east and west tabs are symmetrically spaced",
    )

    for tab, joint in (
        (north_tab, north_joint),
        (south_tab, south_joint),
        (east_tab, east_joint),
        (west_tab, west_joint),
    ):
        with ctx.pose({joint: OPEN_ANGLE}):
            ctx.expect_gap(
                tab,
                ring_body,
                axis="z",
                positive_elem="grip_rib",
                negative_elem="ring_shell",
                min_gap=0.0045,
                name=f"{tab.name} flick edge lifts clear of the ring face when flicked open",
            )

    with ctx.pose({north_joint: OPEN_ANGLE}):
        ctx.expect_gap(
            south_tab,
            ring_body,
            axis="z",
            positive_elem="paddle",
            negative_elem="ring_shell",
            max_gap=0.0002,
            max_penetration=0.0,
            name="south tab remains closed while north tab moves",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
