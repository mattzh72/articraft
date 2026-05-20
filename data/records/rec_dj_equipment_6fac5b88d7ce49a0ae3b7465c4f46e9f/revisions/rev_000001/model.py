from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_effects_stompbox_board")

    powder_black = model.material("powder_black", color=(0.015, 0.015, 0.018, 1.0))
    tray_blue = model.material("tray_blue", color=(0.05, 0.09, 0.14, 1.0))
    brushed_metal = model.material("brushed_metal", color=(0.68, 0.66, 0.60, 1.0))
    dark_rubber = model.material("dark_rubber", color=(0.01, 0.01, 0.01, 1.0))
    knob_black = model.material("knob_black", color=(0.025, 0.025, 0.027, 1.0))
    pointer_white = model.material("pointer_white", color=(0.92, 0.90, 0.82, 1.0))
    switch_chrome = model.material("switch_chrome", color=(0.78, 0.76, 0.70, 1.0))
    unit_colors = [
        model.material("deep_red", color=(0.55, 0.05, 0.035, 1.0)),
        model.material("acid_green", color=(0.08, 0.42, 0.16, 1.0)),
        model.material("electric_violet", color=(0.23, 0.10, 0.55, 1.0)),
    ]

    tray = model.part("tray")
    tray.visual(Box((0.90, 0.50, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.009)), material=tray_blue, name="floor")
    tray.visual(Box((0.018, 0.50, 0.055)), origin=Origin(xyz=(-0.459, 0.0, 0.0275)), material=powder_black, name="side_wall_0")
    tray.visual(Box((0.018, 0.50, 0.055)), origin=Origin(xyz=(0.459, 0.0, 0.0275)), material=powder_black, name="side_wall_1")
    tray.visual(Box((0.90, 0.018, 0.055)), origin=Origin(xyz=(0.0, -0.259, 0.0275)), material=powder_black, name="front_wall")
    tray.visual(Box((0.90, 0.018, 0.055)), origin=Origin(xyz=(0.0, 0.259, 0.0275)), material=powder_black, name="rear_wall")

    box_w = 0.200
    box_d = 0.250
    box_h = 0.055
    body_top_z = 0.047
    hinge_y = 0.130
    hinge_z = 0.058
    unit_xs = (-0.280, 0.0, 0.280)

    knob_meshes = []
    for i in range(3):
        knob_geom = KnobGeometry(
            0.028,
            0.018,
            body_style="faceted",
            base_diameter=0.030,
            top_diameter=0.022,
            edge_radius=0.0008,
            grip=KnobGrip(style="ribbed", count=14, depth=0.0008, width=0.0014),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0005, angle_deg=0.0),
            center=False,
        )
        knob_meshes.append(mesh_from_geometry(knob_geom, f"control_knob_mesh_{i}"))

    for i, x in enumerate(unit_xs):
        # Low fixed bracket and hinge hardware, authored as part of the tray.
        tray.visual(Box((0.240, 0.300, 0.006)), origin=Origin(xyz=(x, -0.005, 0.021)), material=brushed_metal, name=f"bracket_plate_{i}")
        for side, dx in enumerate((-0.085, 0.085)):
            tray.visual(Box((0.042, 0.014, 0.040)), origin=Origin(xyz=(x + dx, hinge_y, 0.038)), material=brushed_metal, name=f"hinge_cheek_{i}_{side}")
            tray.visual(
                Cylinder(radius=0.010, length=0.040),
                origin=Origin(xyz=(x + dx, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=brushed_metal,
                name=f"bracket_barrel_{i}_{side}",
            )
        tray.visual(
            Cylinder(radius=0.0035, length=0.225),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=switch_chrome,
            name=f"hinge_pin_{i}",
        )
        for dx in (-0.060, 0.060):
            tray.visual(
                Cylinder(radius=0.010, length=0.023),
                origin=Origin(xyz=(x + dx, -0.105, 0.0355)),
                material=dark_rubber,
                name=f"rubber_stop_{i}_{0 if dx < 0 else 1}",
            )

        body = model.part(f"stompbox_{i}")
        body_geom = ExtrudeGeometry.from_z0(
            rounded_rect_profile(box_w, box_d, 0.018, corner_segments=8),
            box_h,
            cap=True,
            closed=True,
        ).translate(0.0, -(box_d / 2.0 + 0.010), -0.010)
        body.visual(mesh_from_geometry(body_geom, f"rounded_body_{i}"), material=unit_colors[i], name="body_shell")
        body.visual(Box((0.150, 0.088, 0.002)), origin=Origin(xyz=(0.0, -0.115, 0.046)), material=pointer_white, name="label_panel")
        body.visual(Box((0.132, 0.008, 0.006)), origin=Origin(xyz=(0.0, -0.0085, -0.003)), material=unit_colors[i], name="hinge_leaf")
        body.visual(
            Cylinder(radius=0.010, length=0.120),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=unit_colors[i],
            name="body_hinge",
        )
        for side, sx in enumerate((-1.0, 1.0)):
            body.visual(
                Cylinder(radius=0.012, length=0.008),
                origin=Origin(xyz=(sx * (box_w / 2.0 + 0.001), -0.165, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=powder_black,
                name=f"audio_jack_{side}",
            )
        body.visual(Box((0.108, 0.074, 0.002)), origin=Origin(xyz=(0.0, -0.205, -0.011)), material=dark_rubber, name="battery_hatch")

        model.articulation(
            f"box_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=tray,
            child=body,
            origin=Origin(xyz=(x, hinge_y, hinge_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.10),
        )

        knob_positions = [(-0.055, -0.080), (0.000, -0.080), (0.055, -0.080)]
        for j, (kx, ky) in enumerate(knob_positions):
            knob = model.part(f"knob_{i}_{j}")
            knob.visual(knob_meshes[i], material=knob_black, name="knob_cap")
            knob.visual(Box((0.003, 0.016, 0.0012)), origin=Origin(xyz=(0.0, 0.004, 0.0186)), material=pointer_white, name="pointer_line")
            model.articulation(
                f"knob_spin_{i}_{j}",
                ArticulationType.CONTINUOUS,
                parent=body,
                child=knob,
                origin=Origin(xyz=(kx, ky, body_top_z)),
                axis=(0.0, 0.0, 1.0),
                motion_limits=MotionLimits(effort=0.25, velocity=8.0),
            )

        footswitch = model.part(f"footswitch_{i}")
        footswitch.visual(Cylinder(radius=0.018, length=0.012), origin=Origin(xyz=(0.0, 0.0, 0.006)), material=switch_chrome, name="switch_cap")
        footswitch.visual(Cylinder(radius=0.010, length=0.006), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=dark_rubber, name="rubber_top")
        model.articulation(
            f"switch_press_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=footswitch,
            origin=Origin(xyz=(0.0, -0.185, 0.045)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tray = object_model.get_part("tray")
    hinge_joints = [object_model.get_articulation(f"box_hinge_{i}") for i in range(3)]
    knob_joints = [object_model.get_articulation(f"knob_spin_{i}_{j}") for i in range(3) for j in range(3)]

    ctx.check(
        "three hinged effect units",
        len(hinge_joints) == 3 and all(j.articulation_type == ArticulationType.REVOLUTE for j in hinge_joints),
        details=f"hinges={[(j.name, j.articulation_type) for j in hinge_joints]}",
    )
    ctx.check(
        "all knobs spin continuously",
        len(knob_joints) == 9 and all(j.articulation_type == ArticulationType.CONTINUOUS for j in knob_joints),
        details=f"knobs={[(j.name, j.articulation_type) for j in knob_joints]}",
    )

    for i, hinge in enumerate(hinge_joints):
        body = object_model.get_part(f"stompbox_{i}")
        ctx.allow_overlap(
            tray,
            body,
            elem_a=f"hinge_pin_{i}",
            elem_b="body_hinge",
            reason="The visible steel hinge pin is intentionally captured inside the stomp-box hinge barrel.",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a=f"hinge_pin_{i}",
            elem_b="body_hinge",
            min_overlap=0.110,
            name=f"hinge pin spans body barrel {i}",
        )
        ctx.expect_gap(
            body,
            tray,
            axis="z",
            positive_elem="body_shell",
            negative_elem=f"rubber_stop_{i}_0",
            min_gap=0.0,
            max_gap=0.004,
            name=f"stompbox {i} rests just above front stop",
        )

        closed_aabb = ctx.part_world_aabb(body)
        with ctx.pose({hinge: 1.0}):
            open_aabb = ctx.part_world_aabb(body)
        ctx.check(
            f"stompbox {i} pivots upward for battery access",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.080,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
