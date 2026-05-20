from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PANEL_W = 0.78
PANEL_H = 0.40
PANEL_T = 0.028
OPEN_W = 0.52
OPEN_H = 0.24

BOX_DEPTH = 0.19
BOX_WALL = 0.018
BACK_T = 0.014

DOOR_W = 0.49
DOOR_H = 0.226
DOOR_T = 0.018
HINGE_RADIUS = 0.008
HINGE_Y = -PANEL_T / 2.0 - HINGE_RADIUS - 0.002
HINGE_Z = OPEN_H / 2.0 + 0.014
HINGE_TO_DOOR_TOP = 0.014


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aircraft_glove_compartment")

    fascia_mat = model.material("warm_gray_molded_fascia", rgba=(0.46, 0.48, 0.48, 1.0))
    dark_mat = model.material("dark_cavity_shadow", rgba=(0.035, 0.038, 0.040, 1.0))
    seal_mat = model.material("black_rubber_seal", rgba=(0.005, 0.006, 0.006, 1.0))
    door_mat = model.material("slightly_lighter_door", rgba=(0.56, 0.58, 0.58, 1.0))
    metal_mat = model.material("brushed_latch_metal", rgba=(0.72, 0.70, 0.66, 1.0))
    label_mat = model.material("engraved_black_label", rgba=(0.01, 0.012, 0.014, 1.0))

    fascia = model.part("fascia")
    fascia_bezel = BezelGeometry(
        (OPEN_W, OPEN_H),
        (PANEL_W, PANEL_H),
        PANEL_T,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.018,
        outer_corner_radius=0.045,
    )
    fascia.visual(
        mesh_from_geometry(fascia_bezel, "fascia_bezel"),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=fascia_mat,
        name="fascia_bezel",
    )

    # A real glove box is a shallow open tray behind the dashboard cutout, not a
    # solid block.  These five walls leave the full front opening clear.
    tray_y = PANEL_T / 2.0 + BOX_DEPTH / 2.0
    fascia.visual(
        Box((BOX_WALL, BOX_DEPTH, OPEN_H + 2.0 * BOX_WALL)),
        origin=Origin(xyz=(OPEN_W / 2.0 + BOX_WALL / 2.0, tray_y, 0.0)),
        material=dark_mat,
        name="side_wall_0",
    )
    fascia.visual(
        Box((BOX_WALL, BOX_DEPTH, OPEN_H + 2.0 * BOX_WALL)),
        origin=Origin(xyz=(-(OPEN_W / 2.0 + BOX_WALL / 2.0), tray_y, 0.0)),
        material=dark_mat,
        name="side_wall_1",
    )
    fascia.visual(
        Box((OPEN_W + 2.0 * BOX_WALL, BOX_DEPTH, BOX_WALL)),
        origin=Origin(xyz=(0.0, tray_y, OPEN_H / 2.0 + BOX_WALL / 2.0)),
        material=dark_mat,
        name="top_wall",
    )
    fascia.visual(
        Box((OPEN_W + 2.0 * BOX_WALL, BOX_DEPTH, BOX_WALL)),
        origin=Origin(xyz=(0.0, tray_y, -(OPEN_H / 2.0 + BOX_WALL / 2.0))),
        material=dark_mat,
        name="bottom_wall",
    )
    fascia.visual(
        Box((OPEN_W + 2.0 * BOX_WALL, BACK_T, OPEN_H + 2.0 * BOX_WALL)),
        origin=Origin(xyz=(0.0, PANEL_T / 2.0 + BOX_DEPTH + BACK_T / 2.0, 0.0)),
        material=dark_mat,
        name="back_wall",
    )

    # Thin proud rubber gasket around the cutout and four flush aircraft-style
    # screw heads make the front read as a molded dashboard fascia.
    gasket_y = -PANEL_T / 2.0 - 0.001
    fascia.visual(
        Box((OPEN_W + 0.036, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, gasket_y, OPEN_H / 2.0 + 0.010)),
        material=seal_mat,
        name="upper_gasket",
    )
    fascia.visual(
        Box((OPEN_W + 0.036, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, gasket_y, -(OPEN_H / 2.0 + 0.010))),
        material=seal_mat,
        name="lower_gasket",
    )
    fascia.visual(
        Box((0.010, 0.002, OPEN_H + 0.020)),
        origin=Origin(xyz=(OPEN_W / 2.0 + 0.010, gasket_y, 0.0)),
        material=seal_mat,
        name="side_gasket_0",
    )
    fascia.visual(
        Box((0.010, 0.002, OPEN_H + 0.020)),
        origin=Origin(xyz=(-(OPEN_W / 2.0 + 0.010), gasket_y, 0.0)),
        material=seal_mat,
        name="side_gasket_1",
    )
    for i, (x, z) in enumerate(
        (
            (-OPEN_W / 2.0 - 0.050, OPEN_H / 2.0 + 0.045),
            (OPEN_W / 2.0 + 0.050, OPEN_H / 2.0 + 0.045),
            (-OPEN_W / 2.0 - 0.050, -OPEN_H / 2.0 - 0.045),
            (OPEN_W / 2.0 + 0.050, -OPEN_H / 2.0 - 0.045),
        )
    ):
        fascia.visual(
            Cylinder(radius=0.009, length=0.003),
            origin=Origin(xyz=(x, -PANEL_T / 2.0 - 0.0015, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=metal_mat,
            name=f"screw_head_{i}",
        )

    # Static hinge knuckles are split along the hinge line so the door knuckles
    # can occupy the intervening gaps without interpenetration.
    for i, (x, length) in enumerate(((-0.205, 0.090), (0.0, 0.090), (0.205, 0.090))):
        fascia.visual(
            Cylinder(radius=HINGE_RADIUS, length=length),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"frame_knuckle_{i}",
        )
        fascia.visual(
            Box((length, 0.010, 0.010)),
            origin=Origin(xyz=(x, HINGE_Y + 0.005, HINGE_Z - 0.006)),
            material=metal_mat,
            name=f"frame_hinge_tab_{i}",
        )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(0.0, 0.0, -(HINGE_TO_DOOR_TOP + DOOR_H / 2.0))),
        material=door_mat,
        name="door_panel",
    )
    door.visual(
        Box((DOOR_W - 0.055, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0 - 0.0015, -0.038)),
        material=fascia_mat,
        name="upper_stiffener",
    )
    door.visual(
        Box((DOOR_W - 0.055, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0 - 0.0015, -(HINGE_TO_DOOR_TOP + DOOR_H - 0.008))),
        material=fascia_mat,
        name="lower_stiffener",
    )
    door.visual(
        Box((0.012, 0.003, DOOR_H - 0.070)),
        origin=Origin(xyz=(-DOOR_W / 2.0 + 0.030, -DOOR_T / 2.0 - 0.0015, -(HINGE_TO_DOOR_TOP + DOOR_H / 2.0))),
        material=fascia_mat,
        name="side_stiffener_0",
    )
    door.visual(
        Box((0.012, 0.003, DOOR_H - 0.070)),
        origin=Origin(xyz=(DOOR_W / 2.0 - 0.030, -DOOR_T / 2.0 - 0.0015, -(HINGE_TO_DOOR_TOP + DOOR_H / 2.0))),
        material=fascia_mat,
        name="side_stiffener_1",
    )
    door.visual(
        Box((0.145, 0.002, 0.020)),
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0 - 0.001, -(HINGE_TO_DOOR_TOP + DOOR_H - 0.080))),
        material=label_mat,
        name="latch_recess",
    )
    for i, (x, length) in enumerate(((-0.105, 0.090), (0.105, 0.090))):
        door.visual(
            Cylinder(radius=HINGE_RADIUS, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"door_knuckle_{i}",
        )
        door.visual(
            Box((length, 0.008, 0.012)),
            origin=Origin(xyz=(x, -0.003, -0.008)),
            material=metal_mat,
            name=f"door_hinge_leaf_{i}",
        )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=fascia,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # The closed door extends downward from the hinge line.  Around -X,
        # positive motion lifts the lower edge upward and out toward the cabin.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=0.0, upper=1.35),
    )

    latch = model.part("latch_handle")
    latch.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="pivot_cap",
    )
    latch.visual(
        Box((0.120, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
        material=metal_mat,
        name="handle_bar",
    )
    latch.visual(
        Box((0.036, 0.003, 0.006)),
        origin=Origin(xyz=(0.0, -0.014, 0.0)),
        material=label_mat,
        name="grip_slot",
    )

    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.0, -DOOR_T / 2.0, -(HINGE_TO_DOOR_TOP + DOOR_H - 0.040))),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fascia = object_model.get_part("fascia")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch_handle")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    def coord(vec, idx: int) -> float:
        try:
            return float(vec[idx])
        except TypeError:
            return float((vec.x, vec.y, vec.z)[idx])

    def span(aabb, idx: int) -> float:
        return coord(aabb[1], idx) - coord(aabb[0], idx)

    ctx.expect_gap(
        fascia,
        door,
        axis="y",
        min_gap=0.0005,
        max_gap=0.004,
        positive_elem="fascia_bezel",
        negative_elem="door_panel",
        name="closed door sits just proud of fascia",
    )
    ctx.expect_overlap(
        door,
        fascia,
        axes="xz",
        min_overlap=0.16,
        elem_a="door_panel",
        elem_b="fascia_bezel",
        name="door covers the glove-box opening footprint",
    )
    ctx.expect_contact(
        latch,
        door,
        elem_a="pivot_cap",
        elem_b="door_panel",
        contact_tol=0.0008,
        name="latch pivot cap is seated on the door face",
    )
    ctx.expect_overlap(
        latch,
        door,
        axes="xz",
        min_overlap=0.020,
        elem_a="pivot_cap",
        elem_b="door_panel",
        name="latch pivot is centered on the lower door panel",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "door rotates upward and outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and coord(open_door_aabb[0], 1) < coord(closed_door_aabb[0], 1) - 0.08
        and coord(open_door_aabb[0], 2) > coord(closed_door_aabb[0], 2) + 0.08,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(latch, elem="handle_bar")
    with ctx.pose({latch_pivot: 0.75}):
        turned_handle_aabb = ctx.part_element_world_aabb(latch, elem="handle_bar")
    ctx.check(
        "latch handle turns on its face pivot",
        closed_handle_aabb is not None
        and turned_handle_aabb is not None
        and span(turned_handle_aabb, 2) > span(closed_handle_aabb, 2) + 0.035
        and span(turned_handle_aabb, 0) < span(closed_handle_aabb, 0) - 0.010,
        details=f"closed={closed_handle_aabb}, turned={turned_handle_aabb}",
    )
    ctx.check(
        "upper hinge and latch axes are horizontal/normal",
        door_hinge.axis == (-1.0, 0.0, 0.0) and latch_pivot.axis == (0.0, -1.0, 0.0),
        details=f"door_axis={door_hinge.axis}, latch_axis={latch_pivot.axis}",
    )

    return ctx.report()


object_model = build_object_model()
