from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
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


BARREL_LENGTH = 1.05
BARREL_RADIUS = 0.28
BARREL_THICKNESS = 0.022
BARREL_CENTER_Z = 0.86
HINGE_Y = BARREL_RADIUS
HINGE_Z = BARREL_CENTER_Z
LID_OPEN_RADIANS = math.radians(95.0)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _barrel_half_shell(
    *,
    length: float,
    radius: float,
    thickness: float,
    angle_start: float,
    angle_end: float,
    y_offset: float,
    z_offset: float,
    segments: int = 40,
) -> MeshGeometry:
    """Build a connected thin-walled half-barrel with filled end plates.

    Angles are measured in the local YZ cross-section: y = r*cos(a),
    z = r*sin(a).  The upper lid is authored in its hinge frame by passing
    y_offset=-radius and z_offset=0; the fixed lower bowl is authored directly
    in the cart frame by passing z_offset=BARREL_CENTER_Z.
    """

    mesh = MeshGeometry()
    inner_radius = radius - thickness
    xs = (-length / 2.0, length / 2.0)
    angles = [
        angle_start + (angle_end - angle_start) * i / segments
        for i in range(segments + 1)
    ]

    def vertex(x: float, r: float, angle: float) -> int:
        return mesh.add_vertex(
            x,
            y_offset + r * math.cos(angle),
            z_offset + r * math.sin(angle),
        )

    outer: list[list[int]] = []
    inner: list[list[int]] = []
    for x in xs:
        outer.append([vertex(x, radius, a) for a in angles])
        inner.append([vertex(x, inner_radius, a) for a in angles])

    # Curved outer and inner shell faces.
    for i in range(segments):
        _add_quad(mesh, outer[0][i], outer[1][i], outer[1][i + 1], outer[0][i + 1])
        _add_quad(mesh, inner[1][i], inner[0][i], inner[0][i + 1], inner[1][i + 1])

    # Annular end walls at the two barrel ends.
    for xi in (0, 1):
        for i in range(segments):
            _add_quad(mesh, outer[xi][i], outer[xi][i + 1], inner[xi][i + 1], inner[xi][i])

    # Narrow radial lips at the two split edges.
    for ai in (0, segments):
        _add_quad(mesh, outer[0][ai], inner[0][ai], inner[1][ai], outer[1][ai])

    # Filled half-disc end plates, sharing vertices with the shell's inner edge
    # so the mesh remains one connected piece.
    for xi, x in enumerate(xs):
        center = mesh.add_vertex(x, y_offset, z_offset)
        for i in range(segments):
            if xi == 0:
                mesh.add_face(center, inner[xi][i + 1], inner[xi][i])
            else:
                mesh.add_face(center, inner[xi][i], inner[xi][i + 1])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barrel_cart_grill")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.012, 1.0))
    seasoned_steel = model.material("seasoned_steel", rgba=(0.06, 0.055, 0.048, 1.0))
    dark_frame = model.material("black_tube_frame", rgba=(0.025, 0.025, 0.022, 1.0))
    warm_wood = model.material("warm_wood", rgba=(0.55, 0.34, 0.16, 1.0))
    shelf_mat = model.material("brushed_side_shelf", rgba=(0.48, 0.48, 0.45, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    rim_mat = model.material("silver_rim", rgba=(0.62, 0.62, 0.58, 1.0))

    cart = model.part("cart")

    seam_gap_angle = 0.018
    lower_shell = _barrel_half_shell(
        length=BARREL_LENGTH,
        radius=BARREL_RADIUS,
        thickness=BARREL_THICKNESS,
        angle_start=math.pi + seam_gap_angle,
        angle_end=2.0 * math.pi - seam_gap_angle,
        y_offset=0.0,
        z_offset=BARREL_CENTER_Z,
    )
    cart.visual(
        mesh_from_geometry(lower_shell, "lower_barrel_shell"),
        material=matte_black,
        name="lower_shell",
    )

    # A simple grate sits just below the split line and is tied into the end plates.
    for i, y in enumerate([-0.20, -0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15, 0.20]):
        cart.visual(
            Cylinder(radius=0.0055, length=BARREL_LENGTH + 0.004),
            origin=Origin(
                xyz=(0.0, y, BARREL_CENTER_Z - 0.018),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=seasoned_steel,
            name=f"grate_rod_{i}",
        )

    # Square-tube cart frame supporting the lower half of the barrel.
    for x in (-0.40, 0.40):
        for y in (-0.20, 0.20):
            cart.visual(
                Box((0.038, 0.038, 0.50)),
                origin=Origin(xyz=(x, y, 0.405)),
                material=dark_frame,
                name=f"leg_{'p' if x > 0 else 'n'}_{'p' if y > 0 else 'n'}",
            )
            cart.visual(
                Box((0.115, 0.055, 0.026)),
                origin=Origin(xyz=(x, y, 0.666)),
                material=dark_frame,
                name=f"saddle_{'p' if x > 0 else 'n'}_{'p' if y > 0 else 'n'}",
            )

    for y in (-0.24, 0.24):
        cart.visual(
            Box((0.88, 0.035, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.43)),
            material=dark_frame,
            name=f"long_rail_{'front' if y < 0 else 'rear'}",
        )
    for x in (-0.40, 0.40):
        cart.visual(
            Box((0.035, 0.52, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.43)),
            material=dark_frame,
            name=f"cross_rail_{'n' if x < 0 else 'p'}",
        )

    cart.visual(
        Box((0.78, 0.42, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=dark_frame,
        name="lower_tray",
    )

    # Fixed axle, with the rotating wheels mounted at the ends.
    cart.visual(
        Cylinder(radius=0.018, length=1.04),
        origin=Origin(xyz=(0.0, 0.35, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seasoned_steel,
        name="wheel_axle",
    )
    for x in (-0.40, 0.40):
        cart.visual(
            Box((0.045, 0.16, 0.040)),
            origin=Origin(xyz=(x, 0.29, 0.18)),
            material=dark_frame,
            name=f"axle_block_{'n' if x < 0 else 'p'}",
        )

    # Side shelf fixed to the cart and braced back to the chamber.
    cart.visual(
        Box((0.42, 0.58, 0.035)),
        origin=Origin(xyz=(0.79, 0.0, 0.84)),
        material=shelf_mat,
        name="side_shelf_top",
    )
    for y in (-0.235, 0.235):
        cart.visual(
            Box((0.17, 0.035, 0.035)),
            origin=Origin(xyz=(0.565, y, 0.82)),
            material=dark_frame,
            name=f"shelf_mount_{'front' if y < 0 else 'rear'}",
        )
        cart.visual(
            Box((0.035, 0.035, 0.50)),
            origin=Origin(xyz=(0.965, y, 0.585)),
            material=dark_frame,
            name=f"shelf_leg_{'front' if y < 0 else 'rear'}",
        )
        cart.visual(
            Box((0.46, 0.030, 0.030)),
            origin=Origin(xyz=(0.72, y, 0.35)),
            material=dark_frame,
            name=f"shelf_lower_brace_{'front' if y < 0 else 'rear'}",
        )

    lid = model.part("lid")
    upper_shell = _barrel_half_shell(
        length=BARREL_LENGTH,
        radius=BARREL_RADIUS,
        thickness=BARREL_THICKNESS,
        angle_start=seam_gap_angle,
        angle_end=math.pi - seam_gap_angle,
        y_offset=-BARREL_RADIUS,
        z_offset=0.0,
    )
    lid.visual(
        mesh_from_geometry(upper_shell, "lid_barrel_shell"),
        material=matte_black,
        name="lid_shell",
    )

    # Front handle: a wooden grip along the long front edge, carried by two
    # standoffs and pads on the lid.
    handle_y = -2.0 * BARREL_RADIUS - 0.082
    for x in (-0.30, 0.30):
        lid.visual(
            Box((0.085, 0.018, 0.080)),
            origin=Origin(xyz=(x, -2.0 * BARREL_RADIUS - 0.006, 0.060)),
            material=seasoned_steel,
            name=f"handle_pad_{'n' if x < 0 else 'p'}",
        )
        lid.visual(
            Cylinder(radius=0.011, length=0.082),
            origin=Origin(
                xyz=(x, -2.0 * BARREL_RADIUS - 0.045, 0.074),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=seasoned_steel,
            name=f"handle_standoff_{'n' if x < 0 else 'p'}",
        )

    lid.visual(
        Cylinder(radius=0.022, length=0.72),
        origin=Origin(xyz=(0.0, handle_y, 0.074), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_wood,
        name="front_handle",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cart,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.5,
            lower=0.0,
            upper=LID_OPEN_RADIANS,
        ),
    )

    tire_geom = TireGeometry(
        0.155,
        0.080,
        inner_radius=0.106,
        tread=TireTread(style="block", depth=0.008, count=18, land_ratio=0.58),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.008, radius=0.003),
    )
    rim_geom = WheelGeometry(
        0.108,
        0.070,
        rim=WheelRim(inner_radius=0.074, flange_height=0.008, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.035,
            width=0.064,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.045, hole_diameter=0.005),
        ),
        face=WheelFace(dish_depth=0.006, front_inset=0.003, rear_inset=0.002),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.014),
        bore=WheelBore(style="round", diameter=0.018),
    )

    for idx, x in enumerate((-0.552, 0.552)):
        wheel = model.part(f"wheel_{idx}")
        wheel_visual_origin = Origin(rpy=(0.0, math.pi, 0.0)) if idx == 0 else Origin()
        wheel.visual(
            mesh_from_geometry(tire_geom, f"wheel_{idx}_tire"),
            origin=wheel_visual_origin,
            material=rubber,
            name="tire",
        )
        wheel.visual(
            mesh_from_geometry(rim_geom, f"wheel_{idx}_rim"),
            origin=wheel_visual_origin,
            material=rim_mat,
            name="rim",
        )
        model.articulation(
            f"wheel_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=cart,
            child=wheel,
            origin=Origin(xyz=(x, 0.35, 0.18)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cart = object_model.get_part("cart")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    ctx.check(
        "lid opens about ninety five degrees",
        lid_hinge.motion_limits is not None
        and abs(lid_hinge.motion_limits.upper - LID_OPEN_RADIANS) < 1.0e-4,
        details=f"upper={getattr(lid_hinge.motion_limits, 'upper', None)}",
    )
    ctx.check(
        "lid hinge axis is longitudinal",
        tuple(round(v, 3) for v in lid_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            cart,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="lower_shell",
            min_gap=0.0,
            max_gap=0.014,
            name="closed lid sits just above the fixed lower barrel",
        )
        ctx.expect_overlap(
            lid,
            cart,
            axes="xy",
            elem_a="lid_shell",
            elem_b="lower_shell",
            min_overlap=0.40,
            name="closed lid covers the cooking chamber footprint",
        )
        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    with ctx.pose({lid_hinge: LID_OPEN_RADIANS}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    ctx.check(
        "lid rotates upward at full travel",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.18,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for idx in (0, 1):
        joint = object_model.get_articulation(f"wheel_{idx}_spin")
        ctx.check(
            f"wheel {idx} spins continuously on axle",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 3) for v in joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    return ctx.report()


object_model = build_object_model()
