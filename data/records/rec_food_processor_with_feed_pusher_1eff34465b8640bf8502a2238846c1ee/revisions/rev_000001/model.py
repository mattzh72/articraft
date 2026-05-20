from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded appliance-body block centered at the origin."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def _annular_cylinder(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(height + 0.004).translate((0.0, 0.0, -0.002))
    return outer.cut(inner)


def _lid_shell() -> cq.Workplane:
    """Thin clear lid shell in the lid joint frame; +X points from rear hinge to bowl center."""
    center_x = 0.18
    cover = _annular_cylinder(0.180, 0.044, 0.018).translate((center_x, 0.0, -0.014))
    skirt = _annular_cylinder(0.151, 0.139, 0.038).translate((center_x, 0.0, -0.050))
    chute = _annular_cylinder(0.055, 0.043, 0.148).translate((center_x, 0.0, 0.002))
    return cover.union(skirt).union(chute)


def _blade_mesh(angle: float = 0.0, z: float = 0.074) -> cq.Workplane:
    """One swept, sharpened blade tab reaching from the hub."""
    blade = (
        cq.Workplane("XY")
        .polyline([(0.022, -0.012), (0.132, -0.028), (0.120, 0.012), (0.030, 0.018)])
        .close()
        .extrude(0.004)
        .translate((0.0, 0.0, z))
    )
    if angle:
        blade = blade.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
    return blade


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_food_processor")

    warm_white = model.material("warm_white", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = model.material("dark_graphite", rgba=(0.04, 0.045, 0.05, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    clear = model.material("clear_polycarbonate", rgba=(0.68, 0.88, 1.0, 0.34))
    smoked = model.material("smoked_clear", rgba=(0.45, 0.58, 0.64, 0.42))
    steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))

    # Low rounded motor base with a raised bowl socket and front control panel.
    base = model.part("motor_base")
    base_body = _rounded_box((0.55, 0.36, 0.13), 0.035).translate((0.0, 0.0, 0.065))
    base.visual(mesh_from_cadquery(base_body, "motor_base_shell"), material=warm_white, name="base_shell")
    base.visual(Cylinder(radius=0.156, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.143)), material=warm_white, name="bowl_socket")
    base.visual(Box((0.008, 0.184, 0.066)), origin=Origin(xyz=(0.279, 0.0, 0.077)), material=dark, name="front_panel")
    base.visual(Box((0.050, 0.030, 0.120)), origin=Origin(xyz=(0.0, 0.174, 0.178)), material=warm_white, name="side_support_0")
    base.visual(Box((0.050, 0.030, 0.120)), origin=Origin(xyz=(0.0, -0.174, 0.178)), material=warm_white, name="side_support_1")
    base.visual(Cylinder(radius=0.010, length=0.052), origin=Origin(xyz=(0.0, 0.195, 0.235), rpy=(-pi / 2, 0.0, 0.0)), material=dark, name="side_pivot_0")
    base.visual(Cylinder(radius=0.010, length=0.052), origin=Origin(xyz=(0.0, -0.195, 0.235), rpy=(-pi / 2, 0.0, 0.0)), material=dark, name="side_pivot_1")
    for y in (-0.17, 0.17):
        base.visual(
            Box((0.080, 0.030, 0.016)),
            origin=Origin(xyz=(-0.05, y, 0.008)),
            material=black,
            name=f"foot_{0 if y < 0 else 1}",
        )

    # Medium, visibly hollow clear prep bowl: thin sidewall, bottom floor, thick rim.
    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.112, 0.155),
            (0.134, 0.176),
            (0.158, 0.315),
            (0.171, 0.430),
            (0.174, 0.438),
        ],
        inner_profile=[
            (0.052, 0.176),
            (0.120, 0.196),
            (0.146, 0.318),
            (0.157, 0.415),
            (0.158, 0.430),
        ],
        segments=72,
        lip_samples=5,
    )
    bowl.visual(mesh_from_geometry(bowl_shell, "clear_bowl_shell"), material=clear, name="bowl_shell")
    bowl.visual(mesh_from_cadquery(_annular_cylinder(0.070, 0.034, 0.030).translate((0.0, 0.0, 0.154)), "bowl_center_boss"), material=smoked, name="center_boss")
    bowl.visual(mesh_from_cadquery(_annular_cylinder(0.184, 0.159, 0.010).translate((0.0, 0.0, 0.428)), "bowl_top_rim"), material=clear, name="top_rim")
    bowl.visual(Box((0.070, 0.020, 0.016)), origin=Origin(xyz=(-0.180, 0.106, 0.423)), material=smoked, name="hinge_bridge_0")
    bowl.visual(Box((0.070, 0.020, 0.016)), origin=Origin(xyz=(-0.180, -0.106, 0.423)), material=smoked, name="hinge_bridge_1")
    bowl.visual(Box((0.024, 0.028, 0.050)), origin=Origin(xyz=(-0.200, 0.106, 0.452)), material=smoked, name="hinge_support_0")
    bowl.visual(Box((0.024, 0.028, 0.050)), origin=Origin(xyz=(-0.200, -0.106, 0.452)), material=smoked, name="hinge_support_1")

    model.articulation("base_to_bowl", ArticulationType.FIXED, parent=base, child=bowl, origin=Origin())

    # Rear-hinged clear lid with a central feed chute. The child frame is the rear hinge line.
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_shell(), "clear_lid_shell"), material=clear, name="lid_shell")
    lid.visual(Cylinder(radius=0.012, length=0.184), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2, 0.0, 0.0)), material=smoked, name="hinge_barrel")
    lid.visual(Box((0.070, 0.160, 0.012)), origin=Origin(xyz=(0.035, 0.0, -0.005)), material=smoked, name="hinge_leaf")
    lid_hinge = model.articulation(
        "bowl_to_lid",
        ArticulationType.REVOLUTE,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(-0.180, 0.0, 0.455)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.6, lower=0.0, upper=1.65),
    )

    # Feed pusher slides inside the lid chute, leaving clear annular clearance.
    pusher = model.part("feed_pusher")
    pusher.visual(Cylinder(radius=0.041, length=0.155), origin=Origin(xyz=(0.0, 0.0, -0.077)), material=smoked, name="pusher_body")
    pusher.visual(Box((0.0040, 0.0020, 0.130)), origin=Origin(xyz=(0.0412, 0.0, -0.072)), material=smoked, name="guide_rib")
    pusher.visual(Cylinder(radius=0.042, length=0.020), origin=Origin(xyz=(0.0, 0.0, 0.010)), material=smoked, name="thumb_cap")
    pusher_slide = model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.180, 0.0, 0.150)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.25, lower=0.0, upper=0.070),
    )

    # Twin side latch arms on matched side pivots.
    for index, side_y in enumerate((0.205, -0.205)):
        inward = -1.0 if side_y > 0.0 else 1.0
        latch = model.part(f"latch_arm_{index}")
        latch.visual(Cylinder(radius=0.018, length=0.014), origin=Origin(rpy=(-pi / 2, 0.0, 0.0)), material=dark, name="pivot_eye")
        latch.visual(Box((0.028, 0.014, 0.210)), origin=Origin(xyz=(0.0, 0.0, 0.116)), material=dark, name="upright")
        latch.visual(Box((0.050, 0.014, 0.024)), origin=Origin(xyz=(0.012, inward * 0.004, 0.226)), material=dark, name="lid_hook")
        latch.visual(Box((0.020, 0.020, 0.022)), origin=Origin(xyz=(0.0, 0.0, 0.210)), material=dark, name="hook_tooth")
        model.articulation(
            f"base_to_latch_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=latch,
            origin=Origin(xyz=(0.0, side_y, 0.235)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=-0.75, upper=0.55),
        )

    # Centerline blade carrier rotates continuously inside the hollow bowl.
    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(Cylinder(radius=0.030, length=0.088), origin=Origin(xyz=(0.0, 0.0, 0.044)), material=dark, name="drive_hub")
    blade_carrier.visual(mesh_from_cadquery(_blade_mesh(0.0, 0.072), "blade_lower"), material=steel, name="blade_lower")
    blade_carrier.visual(mesh_from_cadquery(_blade_mesh(180.0, 0.086), "blade_upper"), material=steel, name="blade_upper")
    blade_spin = model.articulation(
        "bowl_to_blade",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=blade_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=35.0),
    )

    # Two continuously rotating front dials on their own axes.
    dial_meshes = []
    for i in range(2):
        knob = KnobGeometry(
            0.060,
            0.030,
            body_style="skirted",
            top_diameter=0.048,
            edge_radius=0.002,
            grip=KnobGrip(style="fluted", count=18, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        )
        dial_meshes.append(mesh_from_geometry(knob, f"control_dial_{i}"))

    for index, y in enumerate((-0.055, 0.055)):
        dial = model.part(f"dial_{index}")
        dial.visual(
            Cylinder(radius=0.016, length=0.004),
            origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
            material=dark,
            name="dial_stem",
        )
        dial.visual(
            dial_meshes[index],
            origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
            material=dark,
            name="dial_cap",
        )
        model.articulation(
            f"base_to_dial_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=dial,
            origin=Origin(xyz=(0.283, y, 0.077)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("motor_base")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("feed_pusher")
    blade = object_model.get_part("blade_carrier")
    lid_hinge = object_model.get_articulation("bowl_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    blade_spin = object_model.get_articulation("bowl_to_blade")

    for index in range(2):
        latch = object_model.get_part(f"latch_arm_{index}")
        pivot_elem = f"side_pivot_{index}"
        ctx.allow_overlap(
            base,
            latch,
            elem_a=pivot_elem,
            elem_b="pivot_eye",
            reason="The molded side pin intentionally passes through the latch pivot eye.",
        )
        ctx.expect_overlap(
            base,
            latch,
            axes="yz",
            min_overlap=0.010,
            elem_a=pivot_elem,
            elem_b="pivot_eye",
            name=f"latch_{index} captured on side pivot",
        )
        ctx.expect_within(
            base,
            latch,
            axes="xz",
            margin=0.004,
            inner_elem=pivot_elem,
            outer_elem="pivot_eye",
            name=f"latch_{index} pivot pin sits inside eye silhouette",
        )

    ctx.expect_contact(base, bowl, elem_a="bowl_socket", elem_b="bowl_shell", contact_tol=0.004, name="bowl seats on base socket")
    ctx.expect_within(pusher, lid, axes="xy", inner_elem="pusher_body", outer_elem="lid_shell", margin=0.006, name="pusher fits within clear chute")
    ctx.expect_within(blade, bowl, axes="xy", inner_elem="blade_lower", outer_elem="bowl_shell", margin=0.002, name="blade stays inside bowl footprint")

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.20}):
        opened_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge opens lid upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.08,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    rest_pusher = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: 0.060}):
        depressed_pusher = ctx.part_world_position(pusher)
    ctx.check(
        "feed pusher slides downward in chute",
        rest_pusher is not None and depressed_pusher is not None and depressed_pusher[2] < rest_pusher[2] - 0.045,
        details=f"rest={rest_pusher}, depressed={depressed_pusher}",
    )

    ctx.check(
        "blade carrier is continuous on bowl centerline",
        blade_spin.articulation_type == ArticulationType.CONTINUOUS and tuple(blade_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={blade_spin.articulation_type}, axis={blade_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()
