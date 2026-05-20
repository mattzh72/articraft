from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
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
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cylindrical_shell_segment(
    *,
    length: float,
    outer_radius: float,
    wall: float,
    theta0: float,
    theta1: float,
    segments: int,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> MeshGeometry:
    """Thin cylindrical barrel shell segment with its axis along local X."""
    inner_radius = outer_radius - wall
    cx, cy, cz = center
    mesh = MeshGeometry()

    xs = (cx - length / 2.0, cx + length / 2.0)
    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for ix, x in enumerate(xs):
        outer_row: list[int] = []
        inner_row: list[int] = []
        for i in range(segments + 1):
            t = theta0 + (theta1 - theta0) * i / segments
            co, si = math.cos(t), math.sin(t)
            outer_row.append(mesh.add_vertex(x, cy + outer_radius * co, cz + outer_radius * si))
            inner_row.append(mesh.add_vertex(x, cy + inner_radius * co, cz + inner_radius * si))
        outer.append(outer_row)
        inner.append(inner_row)

    for i in range(segments):
        # Outer curved skin.
        mesh.add_face(outer[0][i], outer[1][i], outer[1][i + 1])
        mesh.add_face(outer[0][i], outer[1][i + 1], outer[0][i + 1])
        # Inner curved skin.
        mesh.add_face(inner[0][i + 1], inner[1][i + 1], inner[1][i])
        mesh.add_face(inner[0][i + 1], inner[1][i], inner[0][i])
        # Annular end plates at x-min and x-max.
        mesh.add_face(outer[0][i], outer[0][i + 1], inner[0][i + 1])
        mesh.add_face(outer[0][i], inner[0][i + 1], inner[0][i])
        mesh.add_face(outer[1][i + 1], outer[1][i], inner[1][i])
        mesh.add_face(outer[1][i + 1], inner[1][i], inner[1][i + 1])

    # Radial lips at the two long cut edges.
    for i in (0, segments):
        mesh.add_face(outer[0][i], inner[0][i], inner[1][i])
        mesh.add_face(outer[0][i], inner[1][i], outer[1][i])

    return mesh


def _box_workpiece(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cart_frame_mesh() -> object:
    """A welded wagon-style cart frame made as one connected steel mesh."""
    members: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []

    # Long side rails and cross members.
    for y in (-0.34, 0.34):
        members.append(((1.80, 0.055, 0.055), (0.05, y, 0.515)))
        members.append(((1.64, 0.045, 0.045), (0.05, y, 0.31)))
    for x in (-0.70, 0.80):
        members.append(((0.065, 0.75, 0.055), (x, 0.0, 0.515)))
        members.append(((0.055, 0.70, 0.045), (x, 0.0, 0.31)))

    # Four upright posts tying upper rails, lower rails, and wheel axles together.
    for x in (-0.70, 0.80):
        for y in (-0.34, 0.34):
            members.append(((0.060, 0.060, 0.44), (x, y, 0.385)))

    # Shallow saddles under the main barrel and a support tray under the firebox.
    for x in (-0.55, 0.25):
        members.append(((0.16, 0.76, 0.050), (x, 0.0, 0.565)))
    members.append(((0.58, 0.76, 0.050), (0.78, 0.0, 0.585)))

    frame = _box_workpiece(*members[0])
    for size, center in members[1:]:
        frame = frame.union(_box_workpiece(size, center))
    return frame


def _firebox_body_mesh() -> object:
    """Rounded rectangular firebox with a welded smoke throat toward the barrel."""
    body = (
        cq.Workplane("XY")
        .box(0.48, 0.42, 0.36)
        .edges("|Z")
        .fillet(0.035)
        .translate((0.77, 0.0, 0.78))
    )
    throat = (
        cq.Workplane("XY")
        .box(0.13, 0.27, 0.19)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.475, 0.0, 0.82))
    )
    return body.union(throat)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_smoker_grill")

    black = model.material("seasoned_black_steel", rgba=(0.015, 0.014, 0.012, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.05, 0.055, 0.055, 1.0))
    worn_edge = model.material("worn_gray_edges", rgba=(0.20, 0.20, 0.18, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    rim_mat = model.material("painted_wheel_rim", rgba=(0.12, 0.13, 0.12, 1.0))
    wood = model.material("oiled_wood", rgba=(0.45, 0.24, 0.10, 1.0))

    main_length = 1.20
    main_radius = 0.30
    wall = 0.018
    main_center = (-0.15, 0.0, 0.88)
    hinge_xyz = (main_center[0], main_center[1] + main_radius, main_center[2])

    cart = model.part("cart")
    cart.visual(
        mesh_from_cadquery(_cart_frame_mesh(), "wagon_cart_frame", tolerance=0.003),
        material=dark_steel,
        name="welded_frame",
    )
    for x in (-0.70, 0.80):
        cart.visual(
            Cylinder(radius=0.023, length=1.12),
            origin=Origin(xyz=(x, 0.0, 0.17), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_edge,
            name=f"axle_{0 if x < 0 else 1}",
        )

    main_chamber = model.part("main_chamber")
    main_chamber.visual(
        mesh_from_geometry(
            _cylindrical_shell_segment(
                length=main_length,
                outer_radius=main_radius,
                wall=wall,
                theta0=math.radians(182.0),
                theta1=math.radians(358.0),
                segments=48,
                center=main_center,
            ),
            "main_lower_barrel",
        ),
        material=black,
        name="barrel_tub",
    )
    model.articulation(
        "cart_to_main_chamber",
        ArticulationType.FIXED,
        parent=cart,
        child=main_chamber,
        origin=Origin(),
    )

    main_lid = model.part("main_lid")
    main_lid.visual(
        mesh_from_geometry(
            _cylindrical_shell_segment(
                length=main_length * 0.98,
                outer_radius=main_radius,
                wall=wall,
                theta0=math.radians(2.0),
                theta1=math.radians(178.0),
                segments=48,
                center=(0.0, -main_radius, 0.0),
            ),
            "main_lid_shell",
        ),
        material=black,
        name="lid_shell",
    )
    main_lid.visual(
        Cylinder(radius=0.020, length=main_length * 0.96),
        origin=Origin(xyz=(0.0, 0.012, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edge,
        name="rear_hinge_barrel",
    )
    main_lid.visual(
        Box((main_length * 0.92, 0.032, 0.016)),
        origin=Origin(xyz=(0.0, 0.002, 0.006)),
        material=worn_edge,
        name="rear_hinge_leaf",
    )
    main_lid.visual(
        Box((0.92, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, -0.61, 0.13)),
        material=wood,
        name="wood_grip",
    )
    for x in (-0.34, 0.34):
        main_lid.visual(
            Box((0.045, 0.060, 0.12)),
            origin=Origin(xyz=(x, -0.58, 0.075)),
            material=worn_edge,
            name=f"grip_standoff_{0 if x < 0 else 1}",
        )
    # The stack is on the moving lid shell, as on many small backyard smokers.
    main_lid.visual(
        Cylinder(radius=0.040, length=0.47),
        origin=Origin(xyz=(-0.55, -0.42, 0.515)),
        material=black,
        name="smoke_stack",
    )
    main_lid.visual(
        Cylinder(radius=0.055, length=0.028),
        origin=Origin(xyz=(-0.55, -0.42, 0.280)),
        material=worn_edge,
        name="stack_flange",
    )

    model.articulation(
        "main_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=main_chamber,
        child=main_lid,
        origin=Origin(xyz=hinge_xyz),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    firebox = model.part("firebox")
    firebox.visual(
        mesh_from_cadquery(_firebox_body_mesh(), "firebox_body", tolerance=0.003),
        material=black,
        name="firebox_shell",
    )
    firebox.visual(
        Box((0.060, 0.46, 0.035)),
        origin=Origin(xyz=(0.77, 0.0, 0.975)),
        material=worn_edge,
        name="firebox_top_band",
    )
    model.articulation(
        "main_chamber_to_firebox",
        ArticulationType.FIXED,
        parent=main_chamber,
        child=firebox,
        origin=Origin(),
    )

    firebox_door = model.part("firebox_door")
    door_width = 0.35
    door_height = 0.29
    firebox_door.visual(
        Box((0.030, door_width, door_height)),
        origin=Origin(xyz=(0.016, -door_width / 2.0, 0.0)),
        material=black,
        name="door_panel",
    )
    # Raised perimeter ribs on the door face.
    firebox_door.visual(
        Box((0.018, door_width + 0.020, 0.026)),
        origin=Origin(xyz=(0.038, -door_width / 2.0, door_height / 2.0 + 0.004)),
        material=worn_edge,
        name="door_top_rib",
    )
    firebox_door.visual(
        Box((0.018, door_width + 0.020, 0.026)),
        origin=Origin(xyz=(0.038, -door_width / 2.0, -door_height / 2.0 - 0.004)),
        material=worn_edge,
        name="door_bottom_rib",
    )
    firebox_door.visual(
        Box((0.018, 0.026, door_height + 0.030)),
        origin=Origin(xyz=(0.038, -0.010, 0.0)),
        material=worn_edge,
        name="hinge_side_rib",
    )
    firebox_door.visual(
        Box((0.018, 0.026, door_height + 0.030)),
        origin=Origin(xyz=(0.038, -door_width - 0.010, 0.0)),
        material=worn_edge,
        name="latch_side_rib",
    )
    firebox_door.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(0.036, -door_width * 0.62, -0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_edge,
        name="air_vent_disc",
    )
    firebox_door.visual(
        Box((0.075, 0.032, 0.032)),
        origin=Origin(xyz=(0.066, -door_width * 0.72, 0.095)),
        material=wood,
        name="door_grip",
    )

    model.articulation(
        "firebox_door_hinge",
        ArticulationType.REVOLUTE,
        parent=firebox,
        child=firebox_door,
        origin=Origin(xyz=(1.009, 0.205, 0.78)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.55),
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.124,
            0.060,
            rim=WheelRim(inner_radius=0.082, flange_height=0.010, flange_thickness=0.005),
            hub=WheelHub(radius=0.034, width=0.055, cap_style="domed"),
            spokes=WheelSpokes(style="straight", count=8, thickness=0.006, window_radius=0.012),
            bore=WheelBore(style="round", diameter=0.046),
        ),
        "spoked_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.160,
            0.070,
            inner_radius=0.122,
            tread=TireTread(style="block", depth=0.008, count=22, land_ratio=0.56),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.003),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.008, radius=0.003),
        ),
        "wagon_tire",
    )

    wheel_positions = (
        (-0.70, -0.545, 0.17),
        (-0.70, 0.545, 0.17),
        (0.80, -0.545, 0.17),
        (0.80, 0.545, 0.17),
    )
    for i, xyz in enumerate(wheel_positions):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rim_mat,
            name="rim",
        )
        model.articulation(
            f"wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=cart,
            child=wheel,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_chamber = object_model.get_part("main_chamber")
    main_lid = object_model.get_part("main_lid")
    firebox = object_model.get_part("firebox")
    firebox_door = object_model.get_part("firebox_door")
    cart = object_model.get_part("cart")

    main_hinge = object_model.get_articulation("main_lid_hinge")
    firebox_hinge = object_model.get_articulation("firebox_door_hinge")
    wheel_spin = object_model.get_articulation("wheel_spin_0")

    ctx.allow_overlap(
        cart,
        main_chamber,
        elem_a="welded_frame",
        elem_b="barrel_tub",
        reason="The barrel tub is intentionally seated a few millimeters into the welded cart saddle pads.",
    )
    ctx.expect_overlap(
        cart,
        main_chamber,
        axes="xy",
        elem_a="welded_frame",
        elem_b="barrel_tub",
        min_overlap=0.20,
        name="cart saddles support the barrel footprint",
    )
    ctx.expect_gap(
        main_chamber,
        cart,
        axis="z",
        positive_elem="barrel_tub",
        negative_elem="welded_frame",
        max_penetration=0.040,
        name="barrel only nests shallowly into saddle pads",
    )

    ctx.allow_overlap(
        cart,
        firebox,
        elem_a="welded_frame",
        elem_b="firebox_shell",
        reason="The firebox base sits slightly into the welded support tray on the cart.",
    )
    ctx.expect_overlap(
        cart,
        firebox,
        axes="xy",
        elem_a="welded_frame",
        elem_b="firebox_shell",
        min_overlap=0.20,
        name="cart tray supports the firebox footprint",
    )
    ctx.expect_gap(
        firebox,
        cart,
        axis="z",
        positive_elem="firebox_shell",
        negative_elem="welded_frame",
        max_penetration=0.040,
        name="firebox only nests shallowly into support tray",
    )

    ctx.allow_overlap(
        main_chamber,
        main_lid,
        elem_a="barrel_tub",
        elem_b="rear_hinge_barrel",
        reason="The lid hinge barrel is intentionally captured against the rear edge of the cook chamber tub.",
    )
    ctx.expect_overlap(
        main_chamber,
        main_lid,
        axes="x",
        elem_a="barrel_tub",
        elem_b="rear_hinge_barrel",
        min_overlap=1.0,
        name="main lid hinge barrel runs along chamber length",
    )
    ctx.expect_gap(
        main_lid,
        main_chamber,
        axis="z",
        positive_elem="rear_hinge_barrel",
        negative_elem="barrel_tub",
        max_penetration=0.025,
        name="main hinge barrel is only shallowly nested into rear seam",
    )

    for i in range(4):
        wheel = object_model.get_part(f"wheel_{i}")
        axle_name = "axle_0" if i < 2 else "axle_1"
        ctx.allow_overlap(
            cart,
            wheel,
            elem_a=axle_name,
            elem_b="rim",
            reason="The cart axle is intentionally captured inside the wheel hub bore.",
        )
        ctx.expect_overlap(
            cart,
            wheel,
            axes="y",
            elem_a=axle_name,
            elem_b="rim",
            min_overlap=0.040,
            name=f"wheel_{i} hub remains on its axle",
        )
        ctx.expect_within(
            cart,
            wheel,
            axes="xz",
            inner_elem=axle_name,
            outer_elem="rim",
            margin=0.010,
            name=f"wheel_{i} axle is centered in the hub envelope",
        )

    with ctx.pose({main_hinge: 0.0, firebox_hinge: 0.0}):
        ctx.expect_overlap(
            main_lid,
            main_chamber,
            axes="x",
            min_overlap=1.05,
            name="main lid spans the long cook chamber",
        )
        ctx.expect_overlap(
            firebox_door,
            firebox,
            axes="yz",
            min_overlap=0.20,
            name="firebox door covers the side access opening",
        )

    lid_handle_closed = ctx.part_element_world_aabb(main_lid, elem="wood_grip")
    door_grip_closed = ctx.part_element_world_aabb(firebox_door, elem="door_grip")
    wheel_closed = ctx.part_world_position(object_model.get_part("wheel_0"))

    with ctx.pose({main_hinge: 1.05, firebox_hinge: 1.15, wheel_spin: math.pi / 2.0}):
        lid_handle_open = ctx.part_element_world_aabb(main_lid, elem="wood_grip")
        door_grip_open = ctx.part_element_world_aabb(firebox_door, elem="door_grip")
        wheel_turned = ctx.part_world_position(object_model.get_part("wheel_0"))

    ctx.check(
        "main lid rotates upward about rear longitudinal hinge",
        lid_handle_closed is not None
        and lid_handle_open is not None
        and lid_handle_open[1][2] > lid_handle_closed[1][2] + 0.10,
        details=f"closed={lid_handle_closed}, open={lid_handle_open}",
    )
    ctx.check(
        "firebox door swings outward from side hinge",
        door_grip_closed is not None
        and door_grip_open is not None
        and door_grip_open[1][0] > door_grip_closed[1][0] + 0.08,
        details=f"closed={door_grip_closed}, open={door_grip_open}",
    )
    ctx.check(
        "wheel spins about fixed axle without translating",
        wheel_closed is not None
        and wheel_turned is not None
        and max(abs(a - b) for a, b in zip(wheel_closed, wheel_turned)) < 1e-6,
        details=f"closed={wheel_closed}, turned={wheel_turned}",
    )

    return ctx.report()


object_model = build_object_model()
