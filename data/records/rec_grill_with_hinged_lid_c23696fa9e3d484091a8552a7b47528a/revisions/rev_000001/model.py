from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _arched_lid_mesh(
    *,
    width: float,
    depth: float,
    rise: float,
    thickness: float,
    segments: int = 18,
) -> MeshGeometry:
    """A thin, hollow grill hood shell whose local rear hinge line is y=0,z=0."""
    geom = MeshGeometry()
    half = width * 0.5

    outer: list[tuple[float, float]] = []
    inner: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        # t=0 is rear hinge line, t=1 is the front lower lip.
        y = -depth * t
        crown = rise * math.sin(math.pi * t)
        # A slight crown even near the rear/front gives the cover a formed-metal look.
        z_outer = crown + 0.010 * math.sin(math.pi * t) ** 2
        z_inner = z_outer - thickness
        # Keep both lower lips seated at z=0 while preserving shell thickness nearby.
        if index in (0, segments):
            z_inner = thickness
        outer.append((y, z_outer))
        inner.append((y, z_inner))

    # Closed crescent-like cross-section: outer arch rear-to-front, then inner
    # surface front-to-rear.  Extruding this loop across X leaves the underside
    # visually hollow rather than capping the whole cooking zone with a solid box.
    profile = outer + list(reversed(inner))
    left_ids: list[int] = []
    right_ids: list[int] = []
    for y, z in profile:
        left_ids.append(geom.add_vertex(-half, y, z))
        right_ids.append(geom.add_vertex(half, y, z))

    count = len(profile)
    for i in range(count):
        j = (i + 1) % count
        _add_quad(geom, left_ids[i], right_ids[i], right_ids[j], left_ids[j])

    # End caps close the side plates.
    for side_ids, reverse in ((left_ids, True), (right_ids, False)):
        center_id = geom.add_vertex(-half if reverse else half, -depth * 0.5, rise * 0.32)
        for i in range(count):
            j = (i + 1) % count
            if reverse:
                geom.add_face(center_id, side_ids[j], side_ids[i])
            else:
                geom.add_face(center_id, side_ids[i], side_ids[j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_zone_cart_gas_grill")

    matte_black = model.material("matte_black", rgba=(0.035, 0.037, 0.038, 1.0))
    firebox_black = model.material("firebox_black", rgba=(0.055, 0.058, 0.060, 1.0))
    lid_black = model.material("enameled_lid_black", rgba=(0.018, 0.020, 0.021, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.70, 0.70, 0.66, 1.0))
    warm_steel = model.material("warm_steel", rgba=(0.55, 0.53, 0.48, 1.0))
    rubber = model.material("black_rubber", rgba=(0.010, 0.010, 0.011, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.075, 0.078, 0.080, 1.0))
    red = model.material("red_indicator", rgba=(0.75, 0.06, 0.03, 1.0))

    cart = model.part("cart")

    # Wheeled lower cart and fixed cookbox/control-panel assembly.
    cart.visual(Box((1.36, 0.52, 0.030)), origin=Origin(xyz=(0.0, -0.02, 0.20)), material=matte_black, name="lower_shelf")
    cart.visual(Box((1.32, 0.030, 0.035)), origin=Origin(xyz=(0.0, -0.31, 0.39)), material=matte_black, name="front_rail")
    cart.visual(Box((1.32, 0.030, 0.035)), origin=Origin(xyz=(0.0, 0.25, 0.39)), material=matte_black, name="rear_rail")
    for x in (-0.62, 0.62):
        for y in (-0.27, 0.21):
            cart.visual(
                Box((0.055, 0.055, 0.55)),
                origin=Origin(xyz=(x, y, 0.465)),
                material=matte_black,
                name=f"leg_{'n' if x < 0 else 'p'}_{'f' if y < 0 else 'r'}",
            )

    # Propane cylinder and retaining strap on the shelf, fixed to the cart.
    cart.visual(
        Cylinder(radius=0.135, length=0.42),
        origin=Origin(xyz=(-0.39, -0.03, 0.38)),
        material=warm_steel,
        name="propane_tank",
    )
    cart.visual(
        Cylinder(radius=0.143, length=0.030),
        origin=Origin(xyz=(-0.39, -0.03, 0.56)),
        material=matte_black,
        name="tank_retaining_ring",
    )

    # Main cookbox, split top rim, rear hinge strip, and front stainless controls.
    cart.visual(Box((1.22, 0.62, 0.20)), origin=Origin(xyz=(0.0, 0.00, 0.75)), material=firebox_black, name="cookbox")
    cart.visual(Box((1.26, 0.035, 0.045)), origin=Origin(xyz=(0.0, -0.325, 0.865)), material=firebox_black, name="front_lip")
    cart.visual(Box((1.26, 0.035, 0.045)), origin=Origin(xyz=(0.0, 0.305, 0.865)), material=firebox_black, name="rear_lip")
    cart.visual(Box((0.035, 0.62, 0.045)), origin=Origin(xyz=(-0.63, 0.00, 0.865)), material=firebox_black, name="side_lip_0")
    cart.visual(Box((0.035, 0.62, 0.045)), origin=Origin(xyz=(0.63, 0.00, 0.865)), material=firebox_black, name="side_lip_1")
    cart.visual(Box((0.035, 0.61, 0.065)), origin=Origin(xyz=(0.0, -0.01, 0.885)), material=firebox_black, name="center_divider")
    cart.visual(Box((1.27, 0.035, 0.250)), origin=Origin(xyz=(0.0, -0.342, 0.720)), material=stainless, name="control_panel")
    cart.visual(Box((1.16, 0.030, 0.025)), origin=Origin(xyz=(0.0, -0.365, 0.75)), material=warm_steel, name="control_panel_trim")
    cart.visual(Box((0.30, 0.48, 0.045)), origin=Origin(xyz=(-0.78, -0.02, 0.820)), material=stainless, name="side_shelf_0")
    cart.visual(Box((0.30, 0.48, 0.045)), origin=Origin(xyz=(0.78, -0.02, 0.820)), material=stainless, name="side_shelf_1")
    for x in (-0.78, 0.78):
        cart.visual(Box((0.055, 0.035, 0.34)), origin=Origin(xyz=(x, 0.18, 0.63)), material=matte_black, name=f"shelf_rear_support_{0 if x < 0 else 1}")
        cart.visual(Box((0.055, 0.035, 0.36)), origin=Origin(xyz=(x, -0.24, 0.62)), material=matte_black, name=f"shelf_front_support_{0 if x < 0 else 1}")

    # Two visible cooking grates just inside the divided zones.
    for zone, x_center in enumerate((-0.31, 0.31)):
        cart.visual(Box((0.61, 0.58, 0.012)), origin=Origin(xyz=(x_center, -0.02, 0.852)), material=warm_steel, name=f"grate_frame_{zone}")
        for bar in range(6):
            x = x_center - 0.22 + bar * 0.088
            cart.visual(Box((0.012, 0.42, 0.018)), origin=Origin(xyz=(x, -0.02, 0.865)), material=warm_steel, name=f"grate_bar_{zone}_{bar}")

    # Rear hinge leaves and wheel axle stubs are part of the fixed cart.
    for zone, x_center in enumerate((-0.31, 0.31)):
        cart.visual(
            Cylinder(radius=0.018, length=0.54),
            origin=Origin(xyz=(x_center, 0.350, 0.887), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_steel,
            name=f"rear_hinge_pin_{zone}",
        )
        cart.visual(Box((0.12, 0.055, 0.065)), origin=Origin(xyz=(x_center - 0.19, 0.335, 0.837)), material=firebox_black, name=f"hinge_bracket_{zone}_0")
        cart.visual(Box((0.12, 0.055, 0.065)), origin=Origin(xyz=(x_center + 0.19, 0.335, 0.837)), material=firebox_black, name=f"hinge_bracket_{zone}_1")

    for x, side in ((-0.73, -1.0), (0.73, 1.0)):
        inner_x = x - side * 0.075
        for y in (-0.25, 0.20):
            cart.visual(
                Cylinder(radius=0.020, length=0.075),
                origin=Origin(xyz=((x + inner_x) / 2.0, y, 0.115), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=warm_steel,
                name=f"axle_stub_{'left' if x < 0 else 'right'}_{'front' if y < 0 else 'rear'}",
            )
            cart.visual(
                Box((0.060, 0.036, 0.120)),
                origin=Origin(xyz=(inner_x, y, 0.155)),
                material=matte_black,
                name=f"wheel_fork_{'left' if x < 0 else 'right'}_{'front' if y < 0 else 'rear'}",
            )

    lid_mesh = mesh_from_geometry(
        _arched_lid_mesh(width=0.575, depth=0.630, rise=0.235, thickness=0.028),
        "two_zone_grill_lid_shell",
    )

    for zone, x_center in enumerate((-0.31, 0.31)):
        lid = model.part(f"lid_{zone}")
        lid.visual(lid_mesh, origin=Origin(), material=lid_black, name="hood_shell")
        lid.visual(
            Cylinder(radius=0.014, length=0.48),
            origin=Origin(xyz=(0.0, 0.000, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=warm_steel,
            name="hinge_barrel",
        )
        lid.visual(
            Cylinder(radius=0.018, length=0.36),
            origin=Origin(xyz=(0.0, -0.680, 0.125), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="handle_grip",
        )
        for post_x in (-0.16, 0.16):
            lid.visual(
                Cylinder(radius=0.013, length=0.155),
                origin=Origin(xyz=(post_x, -0.610, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=stainless,
                name=f"handle_post_{0 if post_x < 0 else 1}",
            )
        model.articulation(
            f"cart_to_lid_{zone}",
            ArticulationType.REVOLUTE,
            parent=cart,
            child=lid,
            origin=Origin(xyz=(x_center, 0.305, 0.887)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.3, lower=0.0, upper=1.75),
        )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.042,
            body_style="skirted",
            top_diameter=0.052,
            edge_radius=0.002,
            skirt=KnobSkirt(0.083, 0.009, flare=0.08, chamfer=0.0015),
            grip=KnobGrip(style="fluted", count=18, depth=0.0020),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            bore=KnobBore(style="d_shaft", diameter=0.010, flat_depth=0.002),
            center=False,
        ),
        "grill_fluted_burner_knob",
    )
    for index, x in enumerate((-0.45, -0.15, 0.15, 0.45)):
        knob = model.part(f"burner_knob_{index}")
        knob.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm_steel,
            name="shaft_stem",
        )
        knob.visual(knob_mesh, origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_plastic, name="knob_cap")
        knob.visual(
            Box((0.010, 0.006, 0.030)),
            origin=Origin(xyz=(0.0, -0.044, 0.030)),
            material=red,
            name="pointer_mark",
        )
        model.articulation(
            f"cart_to_burner_knob_{index}",
            ArticulationType.REVOLUTE,
            parent=cart,
            child=knob,
            origin=Origin(xyz=(x, -0.3595, 0.660)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=5.0, lower=-2.4, upper=2.4),
        )

    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.070,
            0.052,
            rim=WheelRim(inner_radius=0.044, flange_height=0.006, flange_thickness=0.004, bead_seat_depth=0.002),
            hub=WheelHub(
                radius=0.020,
                width=0.040,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.030, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.004, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.014),
        ),
        "cart_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.095,
            0.070,
            inner_radius=0.064,
            tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.008, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "cart_wheel_tire",
    )
    for wheel_index, (x, y) in enumerate(((-0.73, -0.25), (-0.73, 0.20), (0.73, -0.25), (0.73, 0.20))):
        wheel = model.part(f"wheel_{wheel_index}")
        wheel.visual(tire_mesh, origin=Origin(), material=rubber, name="tire")
        wheel.visual(rim_mesh, origin=Origin(), material=warm_steel, name="rim")
        model.articulation(
            f"cart_to_wheel_{wheel_index}",
            ArticulationType.CONTINUOUS,
            parent=cart,
            child=wheel,
            origin=Origin(xyz=(x, y, 0.115)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cart = object_model.get_part("cart")

    wheel_specs = (
        (0, "axle_stub_left_front"),
        (1, "axle_stub_left_rear"),
        (2, "axle_stub_right_front"),
        (3, "axle_stub_right_rear"),
    )
    for index, axle_elem in wheel_specs:
        wheel = object_model.get_part(f"wheel_{index}")
        ctx.allow_overlap(
            cart,
            wheel,
            elem_a=axle_elem,
            elem_b="rim",
            reason="The fixed axle stub is intentionally captured inside the rotating wheel hub/rim bore.",
        )
        ctx.expect_overlap(
            cart,
            wheel,
            axes="yz",
            elem_a=axle_elem,
            elem_b="rim",
            min_overlap=0.025,
            name=f"wheel_{index} hub is centered on its axle",
        )
        ctx.expect_overlap(
            cart,
            wheel,
            axes="x",
            elem_a=axle_elem,
            elem_b="rim",
            min_overlap=0.020,
            name=f"wheel_{index} axle remains inserted in hub",
        )

    for zone in (0, 1):
        lid = object_model.get_part(f"lid_{zone}")
        hinge = object_model.get_articulation(f"cart_to_lid_{zone}")
        with ctx.pose({hinge: 0.0}):
            ctx.expect_gap(
                lid,
                cart,
                axis="z",
                positive_elem="hood_shell",
                negative_elem="front_lip",
                max_gap=0.004,
                max_penetration=0.004,
                name=f"lid_{zone} front lip stays seated on cookbox",
            )
            closed_aabb = ctx.part_element_world_aabb(lid, elem="handle_grip")
        with ctx.pose({hinge: 1.25}):
            open_aabb = ctx.part_element_world_aabb(lid, elem="handle_grip")
        ctx.check(
            f"lid_{zone} opens upward about rear hinge",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][2] > closed_aabb[0][2] + 0.25,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    for index in range(4):
        knob = object_model.get_part(f"burner_knob_{index}")
        joint = object_model.get_articulation(f"cart_to_burner_knob_{index}")
        ctx.expect_contact(
            knob,
            cart,
            elem_a="shaft_stem",
            elem_b="control_panel",
            contact_tol=0.002,
            name=f"burner_knob_{index} stem seats in control panel",
        )
        ctx.check(
            f"burner_knob_{index} has rotary travel",
            joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper - joint.motion_limits.lower > 4.0,
            details=f"limits={joint.motion_limits}",
        )

    return ctx.report()


object_model = build_object_model()
