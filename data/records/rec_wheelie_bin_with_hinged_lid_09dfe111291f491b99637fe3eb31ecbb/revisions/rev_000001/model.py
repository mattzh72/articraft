from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireShoulder,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _octagonal_loop(half_x: float, half_y: float, chamfer: float, z: float) -> list[tuple[float, float, float]]:
    """Rounded-rectangle-like loop in object coordinates."""
    c = min(chamfer, half_x * 0.75, half_y * 0.75)
    return [
        (half_x - c, half_y, z),
        (-half_x + c, half_y, z),
        (-half_x, half_y - c, z),
        (-half_x, -half_y + c, z),
        (-half_x + c, -half_y, z),
        (half_x - c, -half_y, z),
        (half_x, -half_y + c, z),
        (half_x, half_y - c, z),
    ]


def _add_quad(faces: list[tuple[int, int, int]], a: int, b: int, c: int, d: int) -> None:
    faces.append((a, b, c))
    faces.append((a, c, d))


def _cap_loop(faces: list[tuple[int, int, int]], loop: list[int], *, reverse: bool = False) -> None:
    for index in range(1, len(loop) - 1):
        tri = (loop[0], loop[index], loop[index + 1])
        faces.append(tuple(reversed(tri)) if reverse else tri)


def _hollow_bin_body_mesh() -> MeshGeometry:
    """Thin, open-topped tapered wheelie-bin tub with a visible inside floor."""
    loops = [
        _octagonal_loop(0.215, 0.225, 0.035, 0.220),  # outside underside
        _octagonal_loop(0.315, 0.325, 0.055, 1.020),  # outside top rim
        _octagonal_loop(0.282, 0.292, 0.045, 0.990),  # inside top lip
        _octagonal_loop(0.175, 0.185, 0.030, 0.275),  # raised inside floor
    ]

    vertices: list[tuple[float, float, float]] = []
    loop_indices: list[list[int]] = []
    for loop in loops:
        indices: list[int] = []
        for vertex in loop:
            indices.append(len(vertices))
            vertices.append(vertex)
        loop_indices.append(indices)

    outer_bottom, outer_top, inner_top, inner_bottom = loop_indices
    faces: list[tuple[int, int, int]] = []
    n = len(outer_bottom)

    for i in range(n):
        j = (i + 1) % n
        _add_quad(faces, outer_bottom[i], outer_bottom[j], outer_top[j], outer_top[i])
        _add_quad(faces, inner_top[i], inner_top[j], inner_bottom[j], inner_bottom[i])
        _add_quad(faces, outer_top[i], outer_top[j], inner_top[j], inner_top[i])
        _add_quad(faces, outer_bottom[j], outer_bottom[i], inner_bottom[i], inner_bottom[j])

    _cap_loop(faces, outer_bottom, reverse=True)
    _cap_loop(faces, inner_bottom)
    return MeshGeometry(vertices=vertices, faces=faces)


def _add_wheel_visuals(part, prefix: str, rubber, dark_plastic, grey_plastic) -> None:
    spin_to_y = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                0.130,
                0.070,
                inner_radius=0.088,
                tread=TireTread(style="circumferential", depth=0.004, count=3),
                grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
                sidewall=TireSidewall(style="rounded", bulge=0.035),
                shoulder=TireShoulder(width=0.005, radius=0.002),
            ),
            f"{prefix}_tire",
        ),
        origin=spin_to_y,
        material=rubber,
        name="tire",
    )
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.092,
                0.058,
                rim=WheelRim(inner_radius=0.052, flange_height=0.007, flange_thickness=0.003),
                hub=WheelHub(radius=0.035, width=0.060, cap_style="flat"),
                face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.010),
                bore=WheelBore(style="round", diameter=0.020),
            ),
            f"{prefix}_wheel",
        ),
        origin=spin_to_y,
        material=grey_plastic,
        name="wheel_disc",
    )
    part.visual(
        Cylinder(radius=0.040, length=0.070),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    bin_green = model.material("recycled_green", rgba=(0.06, 0.36, 0.23, 1.0))
    darker_green = model.material("darker_green", rgba=(0.035, 0.22, 0.15, 1.0))
    rubber = model.material("black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.07, 0.075, 0.075, 1.0))
    grey_plastic = model.material("moulded_grey", rgba=(0.42, 0.44, 0.43, 1.0))
    steel = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))

    body = model.part("bin_body")
    body.visual(
        mesh_from_geometry(_hollow_bin_body_mesh(), "bin_body_shell"),
        material=bin_green,
        name="body_shell",
    )

    # Light outer frame: a pronounced top rim and four simple corner posts.
    body.visual(Box((0.045, 0.700, 0.040)), origin=Origin(xyz=(0.325, 0.0, 1.015)), material=darker_green, name="front_top_frame")
    body.visual(Box((0.045, 0.700, 0.040)), origin=Origin(xyz=(-0.325, 0.0, 1.015)), material=darker_green, name="rear_top_frame")
    body.visual(Box((0.640, 0.045, 0.040)), origin=Origin(xyz=(0.0, 0.335, 1.015)), material=darker_green, name="side_top_frame_0")
    body.visual(Box((0.640, 0.045, 0.040)), origin=Origin(xyz=(0.0, -0.335, 1.015)), material=darker_green, name="side_top_frame_1")
    for index, (x, y) in enumerate(((0.255, 0.265), (0.255, -0.265), (-0.255, 0.265), (-0.255, -0.265))):
        body.visual(Box((0.048, 0.048, 0.760)), origin=Origin(xyz=(x, y, 0.620)), material=darker_green, name=f"corner_post_{index}")

    # Rear axle, hangers, and a simple push handle are fixed to the body.
    body.visual(
        Cylinder(radius=0.024, length=0.680),
        origin=Origin(xyz=(-0.345, 0.0, 0.150), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="axle",
    )
    for index, y in enumerate((-0.245, 0.245)):
        body.visual(Box((0.085, 0.050, 0.170)), origin=Origin(xyz=(-0.302, y, 0.220)), material=darker_green, name=f"axle_hanger_{index}")
        body.visual(Box((0.115, 0.052, 0.045)), origin=Origin(xyz=(-0.282, y, 0.305)), material=darker_green, name=f"hanger_shoulder_{index}")
        body.visual(Box((0.140, 0.046, 0.050)), origin=Origin(xyz=(-0.330, y, 0.790)), material=darker_green, name=f"handle_mount_{index}")
        body.visual(Box((0.045, 0.046, 0.225)), origin=Origin(xyz=(-0.385, y, 0.880)), material=darker_green, name=f"handle_post_{index}")
    body.visual(
        Cylinder(radius=0.023, length=0.620),
        origin=Origin(xyz=(-0.395, 0.0, 0.985), rpy=(pi / 2.0, 0.0, 0.0)),
        material=darker_green,
        name="rear_handle",
    )
    for index, y in enumerate((-0.170, 0.170)):
        body.visual(Box((0.075, 0.060, 0.110)), origin=Origin(xyz=(0.165, y, 0.150)), material=darker_green, name=f"front_foot_{index}")
        body.visual(Box((0.070, 0.050, 0.120)), origin=Origin(xyz=(0.175, y, 0.235)), material=darker_green, name=f"front_foot_boss_{index}")

    # Interleaved hinge barrels on the rear edge.
    for index, y in enumerate((-0.220, 0.220)):
        body.visual(Box((0.060, 0.120, 0.042)), origin=Origin(xyz=(-0.338, y, 1.047)), material=darker_green, name=f"hinge_block_{index}")
        body.visual(
            Cylinder(radius=0.023, length=0.125),
            origin=Origin(xyz=(-0.345, y, 1.075), rpy=(pi / 2.0, 0.0, 0.0)),
            material=darker_green,
            name=f"body_hinge_barrel_{index}",
        )

    lid = model.part("lid")
    lid.visual(Box((0.620, 0.690, 0.035)), origin=Origin(xyz=(0.350, 0.0, -0.006)), material=bin_green, name="lid_panel")
    lid.visual(Box((0.045, 0.660, 0.026)), origin=Origin(xyz=(0.650, 0.0, 0.024)), material=darker_green, name="front_lip")
    lid.visual(Box((0.600, 0.038, 0.026)), origin=Origin(xyz=(0.355, 0.328, 0.024)), material=darker_green, name="side_lip_0")
    lid.visual(Box((0.600, 0.038, 0.026)), origin=Origin(xyz=(0.355, -0.328, 0.024)), material=darker_green, name="side_lip_1")
    lid.visual(Box((0.500, 0.045, 0.018)), origin=Origin(xyz=(0.365, 0.0, 0.018)), material=darker_green, name="center_ridge")
    lid.visual(
        Cylinder(radius=0.021, length=0.315),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=darker_green,
        name="lid_hinge_barrel",
    )
    lid.visual(Box((0.075, 0.145, 0.025)), origin=Origin(xyz=(0.036, 0.0, -0.010)), material=darker_green, name="hinge_leaf")

    wheel_0 = model.part("wheel_0")
    _add_wheel_visuals(wheel_0, "wheel_0", rubber, dark_plastic, grey_plastic)

    wheel_1 = model.part("wheel_1")
    _add_wheel_visuals(wheel_1, "wheel_1", rubber, dark_plastic, grey_plastic)

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.345, 0.0, 1.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel_0,
        origin=Origin(xyz=(-0.345, -0.375, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )
    model.articulation(
        "wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel_1,
        origin=Origin(xyz=(-0.345, 0.375, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bin_body")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("wheel_1_spin")

    ctx.check(
        "required_articulations",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and wheel_0_spin.articulation_type == ArticulationType.CONTINUOUS
        and wheel_1_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="Expected one revolute lid hinge and two continuous wheel spin joints.",
    )
    ctx.check(
        "wheel_axes_follow_rear_axle",
        wheel_0_spin.axis == (0.0, 1.0, 0.0) and wheel_1_spin.axis == (0.0, 1.0, 0.0),
        details=f"axes={wheel_0_spin.axis!r}, {wheel_1_spin.axis!r}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_top_frame",
            min_gap=0.002,
            max_gap=0.025,
            name="closed lid sits just above top frame",
        )
        ctx.expect_overlap(lid, body, axes="xy", elem_a="lid_panel", elem_b="body_shell", min_overlap=0.50, name="lid covers the bin opening")

    closed_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({lid_hinge: 1.35}):
        opened_front = ctx.part_element_world_aabb(lid, elem="front_lip")
    ctx.check(
        "lid opens upward",
        closed_front is not None and opened_front is not None and opened_front[1][2] > closed_front[1][2] + 0.35,
        details=f"closed={closed_front}, opened={opened_front}",
    )

    ctx.expect_contact(wheel_0, body, elem_a="hub", elem_b="axle", contact_tol=0.003, name="wheel_0 sits on axle end")
    ctx.expect_contact(wheel_1, body, elem_a="hub", elem_b="axle", contact_tol=0.003, name="wheel_1 sits on axle end")

    return ctx.report()


object_model = build_object_model()
