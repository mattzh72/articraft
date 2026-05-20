from __future__ import annotations

from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="patio_chest_cooler")

    blue = Material("cooler_blue", color=(0.08, 0.34, 0.62, 1.0))
    white = Material("warm_white_plastic", color=(0.94, 0.92, 0.84, 1.0))
    liner = Material("white_inner_liner", color=(0.88, 0.95, 0.98, 1.0))
    dark = Material("dark_gray_plastic", color=(0.05, 0.055, 0.06, 1.0))
    black = Material("black_rubber", color=(0.01, 0.01, 0.012, 1.0))
    metal = Material("brushed_steel", color=(0.58, 0.60, 0.60, 1.0))

    # Cooler dimensions: X=width, Y=depth (front is -Y, rear is +Y), Z=up.
    width = 1.10
    depth = 0.56
    body_height = 0.42
    wall = 0.070
    leg_height = 0.12
    bottom_thick = 0.060
    body_top_z = leg_height + body_height

    body = model.part("body")

    # The insulated chest is modeled as a real open-top box: bottom plus thick
    # side walls, with the liner and rim kept connected to the shell.
    body.visual(
        Box((width, depth, bottom_thick)),
        origin=Origin(xyz=(0.0, 0.0, leg_height + bottom_thick / 2.0)),
        material=blue,
        name="bottom_shell",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, leg_height + body_height / 2.0)),
        material=blue,
        name="front_wall",
    )
    body.visual(
        Box((width, wall, body_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, leg_height + body_height / 2.0)),
        material=blue,
        name="rear_wall",
    )
    body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, leg_height + body_height / 2.0)),
        material=blue,
        name="side_wall_0",
    )
    body.visual(
        Box((wall, depth, body_height)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, leg_height + body_height / 2.0)),
        material=blue,
        name="side_wall_1",
    )

    rim_h = 0.030
    rim_z = body_top_z - rim_h / 2.0
    body.visual(
        Box((width + 0.055, wall + 0.030, rim_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall / 2.0, rim_z)),
        material=white,
        name="top_rim_front",
    )
    body.visual(
        Box((width + 0.055, wall + 0.030, rim_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, rim_z)),
        material=white,
        name="top_rim_rear",
    )
    body.visual(
        Box((wall + 0.030, depth + 0.055, rim_h)),
        origin=Origin(xyz=(-width / 2.0 + wall / 2.0, 0.0, rim_z)),
        material=white,
        name="top_rim_side_0",
    )
    body.visual(
        Box((wall + 0.030, depth + 0.055, rim_h)),
        origin=Origin(xyz=(width / 2.0 - wall / 2.0, 0.0, rim_z)),
        material=white,
        name="top_rim_side_1",
    )

    # Pale inner liner panels make the hollow insulated cavity read clearly.
    liner_z = leg_height + bottom_thick + 0.150
    liner_h = 0.300
    body.visual(
        Box((width - 2.0 * wall, 0.012, liner_h)),
        origin=Origin(xyz=(0.0, -depth / 2.0 + wall + 0.006, liner_z)),
        material=liner,
        name="inner_front_liner",
    )
    body.visual(
        Box((width - 2.0 * wall, 0.012, liner_h)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall - 0.006, liner_z)),
        material=liner,
        name="inner_rear_liner",
    )
    body.visual(
        Box((0.012, depth - 2.0 * wall, liner_h)),
        origin=Origin(xyz=(-width / 2.0 + wall + 0.006, 0.0, liner_z)),
        material=liner,
        name="inner_side_liner_0",
    )
    body.visual(
        Box((0.012, depth - 2.0 * wall, liner_h)),
        origin=Origin(xyz=(width / 2.0 - wall - 0.006, 0.0, liner_z)),
        material=liner,
        name="inner_side_liner_1",
    )
    body.visual(
        Box((width - 2.0 * wall, depth - 2.0 * wall, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, leg_height + bottom_thick + 0.006)),
        material=liner,
        name="inner_floor_liner",
    )

    # Four short, broad patio feet are fused into the body underside.
    leg_size = (0.120, 0.105, leg_height)
    for i, x in enumerate((-0.420, 0.420)):
        for j, y in enumerate((-0.195, 0.195)):
            body.visual(
                Box(leg_size),
                origin=Origin(xyz=(x, y, leg_height / 2.0)),
                material=dark,
                name=f"leg_{i}_{j}",
            )
            body.visual(
                Box((0.145, 0.130, 0.018)),
                origin=Origin(xyz=(x, y, 0.009)),
                material=black,
                name=f"foot_pad_{i}_{j}",
            )

    # Drain socket ring on the lower front face.
    drain_ring = mesh_from_geometry(TorusGeometry(0.041, 0.005, radial_segments=24, tubular_segments=12), "drain_socket")
    body.visual(
        drain_ring,
        origin=Origin(xyz=(0.0, -depth / 2.0 - 0.003, leg_height + 0.105), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="drain_socket",
    )

    # Two exposed rear hinge assemblies. The body carries the outer knuckles;
    # the lid carries the middle knuckles, all sharing the same hinge axis.
    hinge_y = depth / 2.0 + 0.055
    hinge_z = body_top_z + 0.035
    hinge_centers = (-0.335, 0.335)
    for h, x0 in enumerate(hinge_centers):
        body.visual(
            Box((0.165, 0.014, 0.078)),
            origin=Origin(xyz=(x0, depth / 2.0 + 0.010, body_top_z - 0.055)),
            material=metal,
            name=f"hinge_leaf_{h}",
        )
        for k, xk in enumerate((x0 - 0.055, x0 + 0.055)):
            body.visual(
                Cylinder(radius=0.014, length=0.045),
                origin=Origin(xyz=(xk, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
                material=metal,
                name=f"body_knuckle_{h}_{k}",
            )
            body.visual(
                Box((0.045, 0.040, 0.012)),
                origin=Origin(xyz=(xk, depth / 2.0 + 0.062, hinge_z - 0.011)),
                material=metal,
                name=f"knuckle_tab_{h}_{k}",
            )
        body.visual(
            Cylinder(radius=0.006, length=0.170),
            origin=Origin(xyz=(x0, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name=f"hinge_pin_{h}",
        )
        body.visual(
            Box((0.165, 0.052, 0.030)),
            origin=Origin(xyz=(x0, depth / 2.0 + 0.022, body_top_z - 0.040)),
            material=metal,
            name=f"hinge_standoff_{h}",
        )
        for k, xk in enumerate((x0 - 0.055, x0 + 0.055)):
            body.visual(
                Box((0.045, 0.012, 0.090)),
                origin=Origin(xyz=(xk, depth / 2.0 + 0.049, body_top_z - 0.015)),
                material=metal,
                name=f"hinge_backstrap_{h}_{k}",
            )

    lid = model.part("lid")
    lid_depth = 0.640
    lid_width = 1.180
    lid_thick = 0.080
    # Child frame is on the hinge pin axis. The lid slab begins slightly in
    # front of that axis, leaving clearance for the exposed barrels.
    lid.visual(
        Box((lid_width, lid_depth, lid_thick)),
        origin=Origin(xyz=(0.0, -lid_depth / 2.0 - 0.015, 0.011)),
        material=white,
        name="lid_slab",
    )
    lid.visual(
        Box((0.900, 0.390, 0.018)),
        origin=Origin(xyz=(0.0, -0.345, 0.060)),
        material=white,
        name="raised_lid_panel",
    )
    lid.visual(
        Box((0.920, 0.340, 0.016)),
        origin=Origin(xyz=(0.0, -0.345, -0.035)),
        material=black,
        name="gasket",
    )
    lid.visual(
        Box((0.360, 0.020, 0.032)),
        origin=Origin(xyz=(0.0, -0.645, -0.010)),
        material=dark,
        name="front_grip",
    )
    for h, x0 in enumerate(hinge_centers):
        lid.visual(
            Cylinder(radius=0.0135, length=0.055),
            origin=Origin(xyz=(x0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name=f"lid_knuckle_{h}",
        )
        lid.visual(
            Box((0.050, 0.035, 0.012)),
            origin=Origin(xyz=(x0, -0.019, 0.018)),
            material=metal,
            name=f"lid_hinge_tab_{h}",
        )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.22),
    )

    drain_plug = model.part("drain_plug")
    drain_plug.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black,
        name="plug_stem",
    )
    drain_plug.visual(
        Cylinder(radius=0.030, length=0.016),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="plug_cap",
    )
    drain_plug.visual(
        Box((0.038, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.027, 0.0)),
        material=black,
        name="plug_grip",
    )
    model.articulation(
        "drain_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain_plug,
        origin=Origin(xyz=(0.0, -depth / 2.0, leg_height + 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.12, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    drain_plug = object_model.get_part("drain_plug")
    lid_hinge = object_model.get_articulation("lid_hinge")
    drain_slide = object_model.get_articulation("drain_slide")

    ctx.allow_overlap(
        body,
        drain_plug,
        elem_a="front_wall",
        elem_b="plug_stem",
        reason="The rubber drain plug stem is intentionally press-fit through the simplified solid front-wall socket.",
    )
    for h in (0, 1):
        ctx.allow_overlap(
            body,
            lid,
            elem_a=f"hinge_pin_{h}",
            elem_b=f"lid_knuckle_{h}",
            reason="The steel hinge pin is intentionally captured through the lid's rotating hinge knuckle.",
        )

    with ctx.pose({lid_hinge: 0.0, drain_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_slab",
            negative_elem="top_rim_front",
            max_gap=0.010,
            max_penetration=0.0,
            name="closed lid rests just above the front rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_slab",
            elem_b="top_rim_front",
            min_overlap=0.045,
            name="wide lid covers the cooler rim",
        )
        ctx.expect_within(
            drain_plug,
            body,
            axes="xz",
            inner_elem="plug_stem",
            outer_elem="front_wall",
            margin=0.0,
            name="drain plug stem is centered in the front wall",
        )
        ctx.expect_overlap(
            drain_plug,
            body,
            axes="y",
            elem_a="plug_stem",
            elem_b="front_wall",
            min_overlap=0.028,
            name="plug stem remains press-fit in the wall",
        )
        for h in (0, 1):
            ctx.expect_within(
                body,
                lid,
                axes="yz",
                inner_elem=f"hinge_pin_{h}",
                outer_elem=f"lid_knuckle_{h}",
                margin=0.001,
                name=f"hinge pin {h} is centered in the lid knuckle",
            )
            ctx.expect_overlap(
                body,
                lid,
                axes="x",
                elem_a=f"hinge_pin_{h}",
                elem_b=f"lid_knuckle_{h}",
                min_overlap=0.050,
                name=f"hinge pin {h} passes through the moving knuckle",
            )
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_slab")
        closed_plug_pos = ctx.part_world_position(drain_plug)

    with ctx.pose({lid_hinge: 1.15, drain_slide: 0.018}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_slab")
        pulled_plug_pos = ctx.part_world_position(drain_plug)
        ctx.expect_overlap(
            drain_plug,
            body,
            axes="y",
            elem_a="plug_stem",
            elem_b="front_wall",
            min_overlap=0.010,
            name="pulled drain plug keeps retained insertion",
        )

    ctx.check(
        "lid opens upward on the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.22,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "drain plug slides outward from the front face",
        closed_plug_pos is not None
        and pulled_plug_pos is not None
        and pulled_plug_pos[1] < closed_plug_pos[1] - 0.012,
        details=f"closed={closed_plug_pos}, pulled={pulled_plug_pos}",
    )

    return ctx.report()


object_model = build_object_model()
