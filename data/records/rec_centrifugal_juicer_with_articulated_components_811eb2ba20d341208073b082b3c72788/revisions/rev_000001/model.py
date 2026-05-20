from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _annular_extrude(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    """CadQuery ring with its lower face on z=0."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)


def _clear_lid_shape() -> cq.Workplane:
    """One connected transparent lid mesh: shallow cover, central chute, and hinge lug."""
    side_wall = _annular_extrude(0.133, 0.126, 0.050).translate((0.0, -0.145, 0.000))
    top_plate = _annular_extrude(0.126, 0.038, 0.006).translate((0.0, -0.145, 0.044))
    raised_chute = _annular_extrude(0.038, 0.029, 0.190).translate((0.0, -0.145, 0.044))
    hinge_lug = cq.Workplane("XY").box(0.070, 0.040, 0.014).translate((0.0, -0.018, 0.014))
    return side_wall.union(top_plate).union(raised_chute).union(hinge_lug)


def _lower_bowl_shape() -> cq.Workplane:
    """Stationary clear bowl rim under the hinged lid."""
    side_wall = _annular_extrude(0.136, 0.119, 0.038).translate((0.0, 0.0, 0.145))
    pour_lip = cq.Workplane("XY").box(0.060, 0.040, 0.018).translate((0.0, -0.146, 0.166))
    return side_wall.union(pour_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_centrifugal_juicer")

    dark_plastic = model.material("satin_black_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    graphite = model.material("graphite_control_panel", rgba=(0.09, 0.095, 0.105, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.012, 0.012, 0.013, 1.0))
    stainless = model.material("brushed_stainless_steel", rgba=(0.76, 0.74, 0.70, 1.0))
    clear = model.material("smoky_clear_polycarbonate", rgba=(0.58, 0.78, 0.92, 0.34))
    pusher_mat = model.material("warm_gray_pusher", rgba=(0.72, 0.72, 0.68, 1.0))
    orange = model.material("orange_pointer_mark", rgba=(1.0, 0.45, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.155, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=dark_plastic,
        name="base_shell",
    )
    base.visual(
        Box((0.160, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, -0.158, 0.066)),
        material=graphite,
        name="front_panel",
    )
    base.visual(
        mesh_from_cadquery(_lower_bowl_shape(), "lower_bowl"),
        material=clear,
        name="lower_bowl",
    )
    base.visual(
        Cylinder(radius=0.048, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=stainless,
        name="drive_coupler",
    )
    # Rear hinge ears, carried by small posts on the base shell.
    for i, x in enumerate((-0.058, 0.058)):
        base.visual(
            Box((0.030, 0.020, 0.052)),
            origin=Origin(xyz=(x, 0.145, 0.162)),
            material=dark_plastic,
            name=f"hinge_post_{i}",
        )
        base.visual(
            Cylinder(radius=0.012, length=0.034),
            origin=Origin(xyz=(x, 0.145, 0.185), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_plastic,
            name=f"hinge_ear_{i}",
        )
    for i, x in enumerate((-0.085, 0.085)):
        base.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(xyz=(x, -0.070, 0.005)),
            material=rubber,
            name=f"foot_{i}",
        )

    basket = model.part("basket")
    basket.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.090,
                0.042,
                0.056,
                30,
                blade_thickness=0.0024,
                blade_sweep_deg=22.0,
                backplate=True,
                shroud=True,
                center=False,
            ),
            "perforated_basket",
        ),
        material=stainless,
        name="perforated_basket",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_clear_lid_shape(), "clear_lid"),
        material=clear,
        name="clear_lid",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="hinge_barrel",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.025, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=pusher_mat,
        name="pusher_stem",
    )
    pusher.visual(
        Cylinder(radius=0.040, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.243)),
        material=pusher_mat,
        name="thumb_cap",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.038,
            0.020,
            body_style="skirted",
            top_diameter=0.031,
            skirt=KnobSkirt(0.044, 0.004, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=16, depth=0.0009),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.00045),
            center=False,
        ),
        "control_dial_cap",
    )
    for i, x in enumerate((-0.043, 0.043)):
        dial = model.part(f"dial_{i}")
        dial.visual(
            Cylinder(radius=0.007, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=stainless,
            name="dial_shaft",
        )
        dial.visual(
            knob_mesh,
            material=dark_plastic,
            name="dial_cap",
        )
        dial.visual(
            Box((0.004, 0.018, 0.0015)),
            origin=Origin(xyz=(0.0, 0.008, 0.0215)),
            material=orange,
            name="pointer_mark",
        )
        model.articulation(
            f"base_to_dial_{i}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=dial,
            origin=Origin(xyz=(x, -0.167, 0.064), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.15, velocity=8.0),
        )

    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=60.0),
    )
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.145, 0.185)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, -0.145, 0.000)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=0.090),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    basket_spin = object_model.get_articulation("base_to_basket")

    ctx.check(
        "basket has continuous vertical drive",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(basket_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={basket_spin.articulation_type}, axis={basket_spin.axis}",
    )

    for i in range(2):
        dial = object_model.get_part(f"dial_{i}")
        dial_joint = object_model.get_articulation(f"base_to_dial_{i}")
        ctx.allow_overlap(
            base,
            dial,
            elem_a="front_panel",
            elem_b="dial_shaft",
            reason="Each dial's short metal shaft is intentionally seated into the front control panel.",
        )
        ctx.expect_gap(
            base,
            dial,
            axis="y",
            positive_elem="front_panel",
            negative_elem="dial_shaft",
            max_penetration=0.014,
            name=f"dial_{i} shaft has only shallow panel insertion",
        )
        ctx.check(
            f"dial_{i} rotates on its own shaft",
            dial_joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={dial_joint.articulation_type}",
        )

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_stem",
        outer_elem="clear_lid",
        margin=0.004,
        name="pusher is centered inside the feed chute",
    )
    ctx.expect_overlap(
        pusher,
        lid,
        axes="z",
        elem_a="pusher_stem",
        elem_b="clear_lid",
        min_overlap=0.090,
        name="pusher remains inserted through the chute at rest",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({lid_hinge: 1.0}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({pusher_slide: 0.090}):
        raised_pusher_pos = ctx.part_world_position(pusher)
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_stem",
            outer_elem="clear_lid",
            margin=0.004,
            name="raised pusher stays guided by the chute",
        )
        ctx.expect_overlap(
            pusher,
            lid,
            axes="z",
            elem_a="pusher_stem",
            elem_b="clear_lid",
            min_overlap=0.050,
            name="raised pusher retains insertion in the chute",
        )

    ctx.check(
        "lid hinge opens the clear cover upward",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.050,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "pusher slide raises along the feed chute",
        rest_pusher_pos is not None
        and raised_pusher_pos is not None
        and raised_pusher_pos[2] > rest_pusher_pos[2] + 0.080,
        details=f"rest={rest_pusher_pos}, raised={raised_pusher_pos}",
    )

    ctx.expect_overlap(
        basket,
        base,
        axes="xy",
        elem_a="perforated_basket",
        elem_b="lower_bowl",
        min_overlap=0.080,
        name="spinning basket sits inside the upper chamber footprint",
    )

    return ctx.report()


object_model = build_object_model()
