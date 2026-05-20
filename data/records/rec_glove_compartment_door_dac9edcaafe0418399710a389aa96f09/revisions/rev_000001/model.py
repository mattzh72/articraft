from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


DOOR_WIDTH = 0.500
DOOR_HEIGHT = 0.240
DOOR_THICKNESS = 0.016
HINGE_Z = DOOR_HEIGHT / 2.0
HINGE_Y = 0.0


def _mat(model: ArticulatedObject, name: str, rgba: tuple[float, float, float, float]) -> Material:
    return model.material(name, rgba=rgba)


def _storage_tub_shape() -> cq.Workplane:
    """One-piece open-front glove-box tub in root coordinates."""
    width = 0.492
    height = 0.210
    depth = 0.300
    wall = 0.018
    front_y = -0.034
    center_y = front_y - depth / 2.0

    left_wall = cq.Workplane("XY").box(wall, depth, height).translate(
        (-(width - wall) / 2.0, center_y, 0.0)
    )
    right_wall = cq.Workplane("XY").box(wall, depth, height).translate(
        ((width - wall) / 2.0, center_y, 0.0)
    )
    top_wall = cq.Workplane("XY").box(width, depth, wall).translate(
        (0.0, center_y, (height - wall) / 2.0)
    )
    bottom_wall = cq.Workplane("XY").box(width, depth, wall).translate(
        (0.0, center_y, -(height - wall) / 2.0)
    )
    back_wall = cq.Workplane("XY").box(width, wall, height).translate(
        (0.0, front_y - depth + wall / 2.0, 0.0)
    )

    # A molded front flange ties the storage tub into the back of the fascia.
    top_flange = cq.Workplane("XY").box(width + 0.040, 0.028, 0.018).translate(
        (0.0, front_y + 0.006, 0.145)
    )
    bottom_flange = cq.Workplane("XY").box(width + 0.040, 0.028, 0.018).translate(
        (0.0, front_y + 0.006, -height / 2.0 - 0.006)
    )
    left_flange = cq.Workplane("XY").box(0.018, 0.028, height + 0.048).translate(
        (-width / 2.0 - 0.012, front_y + 0.006, 0.0)
    )
    right_flange = cq.Workplane("XY").box(0.018, 0.028, height + 0.048).translate(
        (width / 2.0 + 0.012, front_y + 0.006, 0.0)
    )

    tub = (
        left_wall.union(right_wall)
        .union(top_wall)
        .union(bottom_wall)
        .union(back_wall)
        .union(top_flange)
        .union(bottom_flange)
        .union(left_flange)
        .union(right_flange)
    )
    return tub


def _door_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT)
    panel = panel.edges("|Y").fillet(0.018)
    return panel.translate((0.0, 0.0, -DOOR_HEIGHT / 2.0))


def _stay_plate_shape() -> cq.Workplane:
    """Flat dog-bone friction stay plate with actual pivot holes, in local stay coordinates."""
    y0, z0 = 0.0, 0.0
    y1, z1 = 0.060, -0.130
    dy, dz = y1 - y0, z1 - z0
    length = math.hypot(dy, dz)
    ny, nz = -dz / length, dy / length
    half_width = 0.0045
    strip = [
        (y0 + ny * half_width, z0 + nz * half_width),
        (y1 + ny * half_width, z1 + nz * half_width),
        (y1 - ny * half_width, z1 - nz * half_width),
        (y0 - ny * half_width, z0 - nz * half_width),
    ]

    def disk(y: float, z: float, radius: float) -> cq.Workplane:
        return cq.Workplane("YZ").center(y, z).circle(radius).extrude(0.006, both=True)

    plate = cq.Workplane("YZ").polyline(strip).close().extrude(0.006, both=True)
    plate = plate.union(disk(y0, z0, 0.012)).union(disk(y1, z1, 0.012))
    # The holes are a hair undersized relative to the simplified cylindrical pins
    # so the compiled model has a real captured contact at the pivots.
    pivot_hole = cq.Workplane("YZ").center(y0, z0).circle(0.0067).extrude(0.016, both=True)
    door_hole = cq.Workplane("YZ").center(y1, z1).circle(0.0038).extrude(0.016, both=True)
    return plate.cut(pivot_hole).cut(door_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_dashboard_glove_box")

    fascia_mat = _mat(model, "soft_charcoal_dashboard", (0.060, 0.064, 0.067, 1.0))
    door_mat = _mat(model, "slightly_lighter_door", (0.095, 0.099, 0.103, 1.0))
    dark_mat = _mat(model, "black_plastic", (0.010, 0.011, 0.012, 1.0))
    tub_mat = _mat(model, "dark_tub_plastic", (0.030, 0.033, 0.034, 1.0))
    metal_mat = _mat(model, "brushed_stay_metal", (0.63, 0.63, 0.60, 1.0))

    fascia = model.part("fascia")

    fascia_bezel = BezelGeometry(
        opening_size=(0.525, 0.285),
        outer_size=(0.720, 0.400),
        depth=0.040,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.020,
        outer_corner_radius=0.035,
    )
    fascia.visual(
        mesh_from_geometry(fascia_bezel, "dashboard_fascia"),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fascia_mat,
        name="fascia_surround",
    )
    fascia.visual(
        mesh_from_cadquery(_storage_tub_shape(), "storage_tub", tolerance=0.0008),
        material=tub_mat,
        name="storage_tub",
    )

    # Concealed upper hinge rail and the fixed support pivots are molded into the fixed fascia/tub assembly.
    fascia.visual(
        Cylinder(radius=0.006, length=0.470),
        origin=Origin(xyz=(0.0, -0.020, HINGE_Z - 0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="upper_hinge_pin",
    )
    for side, x in enumerate((-0.227, 0.227)):
        fascia.visual(
            Box((0.016, 0.020, 0.030)),
            origin=Origin(xyz=(x, -0.026, HINGE_Z + 0.006)),
            material=tub_mat,
            name=f"hinge_carrier_{side}",
        )
    for side, x in enumerate((-0.218, 0.218)):
        boss_x = x - 0.015 if x < 0.0 else x + 0.015
        fascia.visual(
            Cylinder(radius=0.007, length=0.026),
            origin=Origin(xyz=(x, -0.080, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"stay_pivot_{side}",
        )
        fascia.visual(
            Box((0.012, 0.026, 0.040)),
            origin=Origin(xyz=(boss_x, -0.091, 0.060)),
            material=tub_mat,
            name=f"stay_boss_{side}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_panel_shape(), "door_panel", tolerance=0.0008),
        material=door_mat,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.420),
        origin=Origin(xyz=(0.0, -0.020, -0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=door_mat,
        name="hinge_sleeve",
    )
    door.visual(
        Box((0.420, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.014, -0.014)),
        material=door_mat,
        name="hinge_web",
    )
    # A very shallow front styling reveal keeps the door readable while staying flush with the fascia.
    door_reveal = BezelGeometry(
        opening_size=(0.410, 0.145),
        outer_size=(0.468, 0.205),
        depth=0.003,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.012,
        outer_corner_radius=0.019,
    )
    door.visual(
        mesh_from_geometry(door_reveal, "door_front_reveal"),
        origin=Origin(
            xyz=(0.0, DOOR_THICKNESS / 2.0 + 0.0015, -DOOR_HEIGHT / 2.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=Material("subtle_reveal", rgba=(0.045, 0.047, 0.050, 1.0)),
        name="front_reveal",
    )
    for side, x in enumerate((-0.218, 0.218)):
        bracket_x = x - 0.015 if x < 0.0 else x + 0.015
        door.visual(
            Cylinder(radius=0.004, length=0.024),
            origin=Origin(xyz=(x, -0.020, -0.190), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"stay_pin_{side}",
        )
        door.visual(
            Box((0.012, 0.010, 0.026)),
            origin=Origin(xyz=(bracket_x, -0.010, -0.190)),
            material=door_mat,
            name=f"stay_bracket_{side}",
        )
        door.visual(
            Box((0.008, 0.010, 0.008)),
            origin=Origin(xyz=(bracket_x, -0.015, -0.190)),
            material=door_mat,
            name=f"stay_pin_neck_{side}",
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=fascia,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=18.0, velocity=1.2),
    )

    latch_knob = model.part("latch_knob")
    knob_geom = KnobGeometry(
        0.054,
        0.020,
        body_style="cylindrical",
        edge_radius=0.0015,
        grip=KnobGrip(style="ribbed", count=22, depth=0.0012, width=0.0015),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0008, angle_deg=90.0),
        center=False,
    )
    latch_knob.visual(
        mesh_from_geometry(knob_geom, "rotary_latch_knob"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="knob_cap",
    )
    latch_knob.visual(
        Cylinder(radius=0.006, length=0.034),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="latch_stem",
    )
    latch_knob.visual(
        Box((0.070, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, -0.029, 0.0)),
        material=metal_mat,
        name="latch_cam",
    )
    model.articulation(
        "knob_axis",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch_knob,
        origin=Origin(xyz=(0.0, DOOR_THICKNESS / 2.0 + 0.001, -0.168)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=math.pi / 2.0, effort=1.5, velocity=5.0),
    )

    stay_plate_mesh = mesh_from_cadquery(_stay_plate_shape(), "side_stay_plate", tolerance=0.0008)
    for side, x in enumerate((-0.218, 0.218)):
        stay = model.part(f"side_stay_{side}")
        stay.visual(
            stay_plate_mesh,
            material=metal_mat,
            name="stay_plate",
        )
        model.articulation(
            f"stay_pivot_{side}",
            ArticulationType.REVOLUTE,
            parent=fascia,
            child=stay,
            origin=Origin(xyz=(x, -0.080, 0.060)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-0.10, upper=0.70, effort=2.5, velocity=2.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fascia = object_model.get_part("fascia")
    door = object_model.get_part("door")
    latch_knob = object_model.get_part("latch_knob")
    stay_0 = object_model.get_part("side_stay_0")
    stay_1 = object_model.get_part("side_stay_1")
    door_hinge = object_model.get_articulation("door_hinge")
    knob_axis = object_model.get_articulation("knob_axis")
    stay_pivot_0 = object_model.get_articulation("stay_pivot_0")

    ctx.allow_overlap(
        door,
        latch_knob,
        elem_a="door_panel",
        elem_b="latch_stem",
        reason="The latch stem intentionally passes through the glove-box door to carry the rear cam.",
    )
    ctx.allow_overlap(
        fascia,
        door,
        elem_a="upper_hinge_pin",
        elem_b="hinge_sleeve",
        reason="The upper hinge pin is intentionally captured inside the door's simplified hinge sleeve.",
    )
    for side, stay in enumerate((stay_0, stay_1)):
        ctx.allow_overlap(
            fascia,
            stay,
            elem_a=f"stay_pivot_{side}",
            elem_b="stay_plate",
            reason="The side-stay support pivot is intentionally captured in the stay plate hole.",
        )
        ctx.allow_overlap(
            door,
            stay,
            elem_a=f"stay_pin_{side}",
            elem_b="stay_plate",
            reason="The door-side stay pin is intentionally captured in the stay plate hole.",
        )
    ctx.expect_overlap(
        latch_knob,
        door,
        axes="xz",
        elem_a="latch_stem",
        elem_b="door_panel",
        min_overlap=0.010,
        name="latch stem crosses the door skin",
    )
    ctx.expect_gap(
        latch_knob,
        door,
        axis="y",
        positive_elem="latch_stem",
        negative_elem="door_panel",
        max_penetration=0.026,
        name="latch stem penetration is local",
    )
    ctx.expect_overlap(
        fascia,
        door,
        axes="x",
        elem_a="upper_hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.35,
        name="upper hinge spans most of the door width",
    )
    ctx.expect_gap(
        fascia,
        door,
        axis="y",
        positive_elem="upper_hinge_pin",
        negative_elem="hinge_sleeve",
        max_penetration=0.015,
        name="hinge pin capture is local",
    )
    for side, stay in enumerate((stay_0, stay_1)):
        ctx.expect_overlap(
            fascia,
            stay,
            axes="x",
            elem_a=f"stay_pivot_{side}",
            elem_b="stay_plate",
            min_overlap=0.010,
            name=f"side stay {side} wraps fixed support pin",
        )
        ctx.expect_overlap(
            door,
            stay,
            axes="x",
            elem_a=f"stay_pin_{side}",
            elem_b="stay_plate",
            min_overlap=0.010,
            name=f"side stay {side} wraps door pin",
        )

    ctx.expect_overlap(
        door,
        fascia,
        axes="xz",
        elem_a="door_panel",
        elem_b="fascia_surround",
        min_overlap=0.20,
        name="flush door fills the dashboard opening footprint",
    )
    ctx.expect_overlap(
        door,
        fascia,
        axes="xz",
        elem_a="door_panel",
        elem_b="storage_tub",
        min_overlap=0.18,
        name="storage tub sits directly behind the door",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.05}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door rotates upward and outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10
        and open_door_aabb[0][2] > closed_door_aabb[0][2] + 0.06,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    cam_closed = ctx.part_element_world_aabb(latch_knob, elem="latch_cam")
    with ctx.pose({knob_axis: math.pi / 2.0}):
        cam_turned = ctx.part_element_world_aabb(latch_knob, elem="latch_cam")
    ctx.check(
        "rotary latch cam turns ninety degrees",
        cam_closed is not None
        and cam_turned is not None
        and (cam_closed[1][0] - cam_closed[0][0]) > 0.055
        and (cam_turned[1][2] - cam_turned[0][2]) > 0.055,
        details=f"closed={cam_closed}, turned={cam_turned}",
    )

    stay_closed = ctx.part_world_aabb(stay_0)
    with ctx.pose({stay_pivot_0: 0.45}):
        stay_moved = ctx.part_world_aabb(stay_0)
    ctx.check(
        "side stay rotates on support pivot",
        stay_closed is not None
        and stay_moved is not None
        and stay_moved[1][1] > stay_closed[1][1] + 0.030
        and stay_moved[1][2] > stay_closed[1][2] + 0.020,
        details=f"closed={stay_closed}, moved={stay_moved}",
    )
    ctx.expect_origin_distance(
        stay_0,
        stay_1,
        axes="x",
        min_dist=0.42,
        name="friction stays are mounted on opposite side walls",
    )

    return ctx.report()


object_model = build_object_model()
