from __future__ import annotations

import math

import cadquery as cq

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
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WIDTH = 0.66
DEPTH = 0.42
BODY_HEIGHT = 0.35
BODY_Z0 = 0.08
LID_HEIGHT = 0.12
LID_Z0 = BODY_Z0 + BODY_HEIGHT


def _safe_fillet(workplane, selector: str, radius: float):
    try:
        return workplane.edges(selector).fillet(radius)
    except Exception:
        return workplane


def _lower_tub_shape():
    """Open-top molded lower compartment with thick walls and radiused corners."""
    wall = 0.030
    outer = cq.Workplane("XY").box(WIDTH, DEPTH, BODY_HEIGHT).translate((0.0, 0.0, BODY_HEIGHT / 2.0))
    outer = _safe_fillet(outer, "|Z", 0.035)

    inner_h = BODY_HEIGHT + 0.05
    cutter = (
        cq.Workplane("XY")
        .box(WIDTH - 2.0 * wall, DEPTH - 2.0 * wall, inner_h)
        .translate((0.0, 0.0, wall + inner_h / 2.0))
    )
    return outer.cut(cutter)


def _lid_shell_shape():
    """Separate upper lid shell with two shallow organizer-cover recesses and a front latch pocket."""
    shell = cq.Workplane("XY").box(WIDTH, DEPTH, LID_HEIGHT).translate((0.0, 0.0, LID_HEIGHT / 2.0))
    shell = _safe_fillet(shell, "|Z", 0.035)

    # Twin shallow pockets in the lid top for clear organizer doors.
    for cx in (-0.145, 0.145):
        pocket = (
            cq.Workplane("XY")
            .box(0.270, 0.290, 0.040)
            .translate((cx, 0.000, LID_HEIGHT - 0.014 + 0.020))
        )
        shell = shell.cut(pocket)

    # Push-button latch cavity in the nose of the lid.
    latch_slot = (
        cq.Workplane("XY")
        .box(0.135, 0.070, 0.058)
        .translate((0.0, -DEPTH / 2.0, 0.055))
    )
    return shell.cut(latch_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="molded_rolling_toolbox")

    charcoal = model.material("charcoal_molded_plastic", rgba=(0.09, 0.10, 0.11, 1.0))
    dark = model.material("dark_recess_plastic", rgba=(0.025, 0.028, 0.030, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    steel = model.material("brushed_steel", rgba=(0.64, 0.66, 0.65, 1.0))
    clear_blue = model.material("smoky_clear_polycarbonate", rgba=(0.45, 0.72, 0.95, 0.38))
    latch_yellow = model.material("yellow_latch_button", rgba=(0.95, 0.68, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_lower_tub_shape(), "lower_compartment_shell", tolerance=0.002),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z0)),
        material=charcoal,
        name="lower_shell",
    )
    body.visual(
        Box((0.74, 0.46, 0.080)),
        origin=Origin(xyz=(0.0, -0.010, 0.040)),
        material=charcoal,
        name="broad_base",
    )
    body.visual(
        Box((0.62, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, -0.235, 0.090)),
        material=black_rubber,
        name="front_bumper",
    )

    # Rear trolley-handle guide channels molded into the back of the box.
    for idx, x in enumerate((-0.220, 0.220)):
        body.visual(
            Box((0.060, 0.008, 0.440)),
            origin=Origin(xyz=(x, 0.214, 0.280)),
            material=charcoal,
            name=f"guide_{idx}_spine",
        )
        body.visual(
            Box((0.008, 0.036, 0.440)),
            origin=Origin(xyz=(x - 0.014, 0.236, 0.280)),
            material=charcoal,
            name=f"guide_{idx}_flange_0",
        )
        body.visual(
            Box((0.008, 0.036, 0.440)),
            origin=Origin(xyz=(x + 0.014, 0.236, 0.280)),
            material=charcoal,
            name=f"guide_{idx}_flange_1",
        )

    # Short axle stubs visibly carry the main transport wheels without piercing the tires.
    body.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(-0.3425, 0.160, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_stub_0",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(0.3425, 0.160, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle_stub_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "upper_lid_shell", tolerance=0.002),
        origin=Origin(),
        material=charcoal,
        name="lid_shell",
    )
    # Dark floors emphasize that the transparent organizer doors sit in recessed bins.
    for idx, x in enumerate((-0.145, 0.145)):
        lid.visual(
            Box((0.245, 0.262, 0.004)),
            origin=Origin(xyz=(x, 0.000, 0.108)),
            material=dark,
            name=f"organizer_floor_{idx}",
        )
        # Small hinge knuckle pads on the lid side of each clear cover.
        for pad_idx, px in enumerate((x - 0.098, x + 0.098)):
            lid.visual(
                Box((0.025, 0.018, 0.018)),
                origin=Origin(xyz=(px, 0.151, 0.119)),
                material=charcoal,
                name=f"cover_{idx}_hinge_pad_{pad_idx}",
            )

    model.articulation(
        "body_to_lid",
        ArticulationType.FIXED,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_Z0)),
    )

    # Recessed telescoping trolley handle: twin metal rails joined by a molded grip.
    handle = model.part("trolley_handle")
    for idx, x in enumerate((-0.220, 0.220)):
        handle.visual(
            Cylinder(radius=0.010, length=0.520),
            origin=Origin(xyz=(x, 0.0, -0.210)),
            material=steel,
            name=f"rail_{idx}",
        )
    handle.visual(
        Cylinder(radius=0.018, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, 0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="top_grip",
    )
    handle.visual(
        Box((0.160, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=black_rubber,
        name="grip_pad",
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.242, 0.470)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=110.0, velocity=0.35, lower=0.0, upper=0.430),
    )

    # Twin transparent organizer covers, each hinged at the rear edge of its lid pocket.
    for idx, x in enumerate((-0.145, 0.145)):
        cover = model.part(f"cover_{idx}")
        cover.visual(
            Box((0.250, 0.265, 0.010)),
            origin=Origin(xyz=(0.0, -0.1325, -0.003)),
            material=clear_blue,
            name="clear_panel",
        )
        cover.visual(
            Cylinder(radius=0.007, length=0.250),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=clear_blue,
            name="hinge_barrel",
        )
        cover.visual(
            Box((0.070, 0.016, 0.012)),
            origin=Origin(xyz=(0.0, -0.270, 0.002)),
            material=clear_blue,
            name="front_lip",
        )
        model.articulation(
            f"cover_{idx}_hinge",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=cover,
            origin=Origin(xyz=(x, 0.135, 0.118)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.35),
        )

    # Front push-button latch mounted in the nose slot of the lid.
    latch = model.part("latch_button")
    latch.visual(
        Box((0.155, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=latch_yellow,
        name="button_face",
    )
    latch.visual(
        Box((0.085, 0.024, 0.026)),
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
        material=latch_yellow,
        name="button_stem",
    )
    model.articulation(
        "latch_press",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=latch,
        origin=Origin(xyz=(0.0, -DEPTH / 2.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.15, lower=0.0, upper=0.018),
    )

    tire = TireGeometry(
        0.085,
        0.070,
        inner_radius=0.058,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
        tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.54),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.008, radius=0.003),
    )
    wheel_core = WheelGeometry(
        0.059,
        0.064,
        rim=WheelRim(inner_radius=0.040, flange_height=0.005, flange_thickness=0.003),
        hub=WheelHub(radius=0.023, width=0.050, cap_style="domed"),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.009),
        bore=WheelBore(style="round", diameter=0.018),
    )
    tire_mesh = mesh_from_geometry(tire, "utility_tire")
    wheel_mesh = mesh_from_geometry(wheel_core, "wheel_core")
    for idx, x in enumerate((-0.405, 0.405)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(tire_mesh, material=black_rubber, name="tire")
        wheel.visual(wheel_mesh, material=steel, name="hub")
        model.articulation(
            f"wheel_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x, 0.160, 0.095)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("trolley_handle")
    latch = object_model.get_part("latch_button")
    handle_slide = object_model.get_articulation("handle_slide")
    latch_press = object_model.get_articulation("latch_press")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="lower_shell",
        max_gap=0.002,
        max_penetration=0.0,
        name="separate lid shell sits on lower compartment rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="lower_shell",
        min_overlap=0.30,
        name="lid footprint covers the deep lower compartment",
    )

    for idx in (0, 1):
        with ctx.pose({handle_slide: 0.430}):
            ctx.expect_overlap(
                handle,
                body,
                axes="z",
                elem_a=f"rail_{idx}",
                elem_b=f"guide_{idx}_spine",
                min_overlap=0.045,
                name=f"extended handle rail {idx} remains captured in rear guide",
            )

    rest_handle = ctx.part_world_position(handle)
    with ctx.pose({handle_slide: 0.430}):
        extended_handle = ctx.part_world_position(handle)
    ctx.check(
        "trolley handle slides upward",
        rest_handle is not None
        and extended_handle is not None
        and extended_handle[2] > rest_handle[2] + 0.38,
        details=f"rest={rest_handle}, extended={extended_handle}",
    )

    rest_latch = ctx.part_world_position(latch)
    with ctx.pose({latch_press: 0.018}):
        pressed_latch = ctx.part_world_position(latch)
    ctx.check(
        "front latch button translates into the lid nose",
        rest_latch is not None
        and pressed_latch is not None
        and pressed_latch[1] > rest_latch[1] + 0.015,
        details=f"rest={rest_latch}, pressed={pressed_latch}",
    )

    for idx in (0, 1):
        cover = object_model.get_part(f"cover_{idx}")
        hinge = object_model.get_articulation(f"cover_{idx}_hinge")
        closed_aabb = ctx.part_world_aabb(cover)
        with ctx.pose({hinge: 1.20}):
            open_aabb = ctx.part_world_aabb(cover)
        ctx.check(
            f"organizer cover {idx} rotates upward on rear hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    for idx in (0, 1):
        spin = object_model.get_articulation(f"wheel_{idx}_spin")
        ctx.check(
            f"transport wheel {idx} has continuous axle spin",
            spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (1.0, 0.0, 0.0),
            details=f"type={spin.articulation_type}, axis={spin.axis}",
        )

    return ctx.report()


object_model = build_object_model()
