from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _bridge_loops(
    geom: MeshGeometry,
    lower_loop: list[int],
    upper_loop: list[int],
    *,
    outward: bool = True,
) -> None:
    count = len(lower_loop)
    for index in range(count):
        next_index = (index + 1) % count
        a = lower_loop[index]
        b = lower_loop[next_index]
        c = upper_loop[next_index]
        d = upper_loop[index]
        if outward:
            _add_quad(geom, a, b, c, d)
        else:
            _add_quad(geom, a, d, c, b)


def _cap_loop(
    geom: MeshGeometry,
    loop: list[int],
    center: tuple[float, float, float],
    *,
    upward: bool,
) -> None:
    center_id = geom.add_vertex(*center)
    count = len(loop)
    for index in range(count):
        next_index = (index + 1) % count
        a = loop[index]
        b = loop[next_index]
        if upward:
            geom.add_face(center_id, a, b)
        else:
            geom.add_face(center_id, b, a)


def _add_loop(
    geom: MeshGeometry,
    profile: list[tuple[float, float]],
    *,
    z: float,
) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y in profile]


def _build_body_shell_mesh() -> object:
    geom = MeshGeometry()
    outer_top_profile = rounded_rect_profile(0.58, 0.74, 0.050, corner_segments=6)
    outer_bottom_profile = rounded_rect_profile(0.43, 0.55, 0.036, corner_segments=6)
    inner_top_profile = rounded_rect_profile(0.52, 0.68, 0.032, corner_segments=6)
    inner_bottom_profile = rounded_rect_profile(0.37, 0.47, 0.020, corner_segments=6)

    outer_bottom = _add_loop(geom, outer_bottom_profile, z=0.16)
    outer_top = _add_loop(geom, outer_top_profile, z=0.90)
    inner_bottom = _add_loop(geom, list(reversed(inner_bottom_profile)), z=0.19)
    inner_top = _add_loop(geom, list(reversed(inner_top_profile)), z=0.865)

    _bridge_loops(geom, outer_bottom, outer_top, outward=True)
    _bridge_loops(geom, inner_bottom, inner_top, outward=True)
    _bridge_loops(geom, outer_top, list(reversed(inner_top)), outward=True)
    _bridge_loops(geom, outer_bottom, list(reversed(inner_bottom)), outward=False)
    _cap_loop(geom, outer_bottom, (0.0, 0.0, 0.16), upward=False)
    _cap_loop(geom, inner_bottom, (0.0, 0.0, 0.19), upward=True)

    return mesh_from_geometry(geom, "wheelie_bin_body_shell")


def _build_wheel_tire_mesh(name: str, *, radius: float, width: float) -> object:
    half_width = width * 0.5
    profile = [
        (radius * 0.56, -half_width * 0.92),
        (radius * 0.82, -half_width),
        (radius * 0.97, -half_width * 0.60),
        (radius, -half_width * 0.16),
        (radius, half_width * 0.16),
        (radius * 0.97, half_width * 0.60),
        (radius * 0.82, half_width),
        (radius * 0.56, half_width * 0.92),
        (radius * 0.46, half_width * 0.44),
        (radius * 0.41, 0.0),
        (radius * 0.46, -half_width * 0.44),
        (radius * 0.56, -half_width * 0.92),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=48).rotate_y(math.pi / 2.0),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    body_plastic = model.material("body_plastic", rgba=(0.13, 0.15, 0.16, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.18, 0.20, 0.21, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.36, 0.38, 0.40, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    wheel_rim = model.material("wheel_rim", rgba=(0.58, 0.60, 0.63, 1.0))

    body = model.part("body")
    body.visual(_build_body_shell_mesh(), material=body_plastic, name="shell")
    body.visual(
        Box((0.62, 0.06, 0.050)),
        origin=Origin(xyz=(0.0, 0.36, 0.875)),
        material=body_plastic,
        name="front_rim_beam",
    )
    body.visual(
        Box((0.62, 0.05, 0.050)),
        origin=Origin(xyz=(0.0, -0.345, 0.875)),
        material=body_plastic,
        name="rear_rim_beam",
    )
    body.visual(
        Box((0.055, 0.74, 0.060)),
        origin=Origin(xyz=(-0.288, 0.0, 0.87)),
        material=body_plastic,
        name="left_rim_post",
    )
    body.visual(
        Box((0.055, 0.74, 0.060)),
        origin=Origin(xyz=(0.288, 0.0, 0.87)),
        material=body_plastic,
        name="right_rim_post",
    )
    body.visual(
        Box((0.42, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, -0.345, 0.79)),
        material=body_plastic,
        name="rear_handle_bridge",
    )
    body.visual(
        Box((0.10, 0.17, 0.18)),
        origin=Origin(xyz=(-0.215, -0.285, 0.18)),
        material=body_plastic,
        name="left_axle_support",
    )
    body.visual(
        Box((0.10, 0.17, 0.18)),
        origin=Origin(xyz=(0.215, -0.285, 0.18)),
        material=body_plastic,
        name="right_axle_support",
    )
    body.visual(
        Box((0.36, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, -0.31, 0.18)),
        material=body_plastic,
        name="axle_cross_member",
    )
    body.visual(
        Box((0.22, 0.12, 0.045)),
        origin=Origin(xyz=(0.0, 0.315, 0.172)),
        material=body_plastic,
        name="front_foot_lip",
    )
    body.visual(
        Box((0.42, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, -0.333, 0.900)),
        material=body_plastic,
        name="hinge_beam",
    )
    body.visual(
        Box((0.48, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, -0.31, 0.15)),
        material=axle_steel,
        name="axle_housing",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.62, 0.78, 0.98)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.49)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.612, 0.685, 0.028)),
        origin=Origin(xyz=(0.0, 0.4175, 0.008)),
        material=lid_plastic,
        name="lid_panel",
    )
    lid.visual(
        Box((0.570, 0.090, 0.028)),
        origin=Origin(xyz=(0.0, 0.045, 0.015)),
        material=lid_plastic,
        name="rear_crown",
    )
    lid.visual(
        Box((0.560, 0.055, 0.090)),
        origin=Origin(xyz=(0.0, -0.0125, -0.020)),
        material=lid_plastic,
        name="rear_skirt",
    )
    lid.visual(
        Box((0.024, 0.640, 0.022)),
        origin=Origin(xyz=(-0.286, 0.368, 0.022)),
        material=lid_plastic,
        name="left_skirt",
    )
    lid.visual(
        Box((0.024, 0.640, 0.022)),
        origin=Origin(xyz=(0.286, 0.368, 0.022)),
        material=lid_plastic,
        name="right_skirt",
    )
    lid.visual(
        Box((0.320, 0.040, 0.040)),
        origin=Origin(xyz=(0.0, 0.735, 0.022)),
        material=lid_plastic,
        name="front_grip",
    )
    for index, x_pos in enumerate((-0.205, 0.0, 0.205)):
        lid.visual(
            Cylinder(radius=0.016, length=0.120),
            origin=Origin(
                xyz=(x_pos, 0.000, 0.000),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=lid_plastic,
            name=f"hinge_barrel_{index}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((0.62, 0.78, 0.10)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.39, -0.01)),
    )

    wheel_radius = 0.125
    wheel_width = 0.055
    tire_mesh = _build_wheel_tire_mesh(
        "wheelie_bin_rear_tire",
        radius=wheel_radius,
        width=wheel_width,
    )
    for side_name, x_pos in (("left_wheel", -0.299), ("right_wheel", 0.299)):
        wheel = model.part(side_name)
        wheel.visual(tire_mesh, material=wheel_rubber, name="tire")
        wheel.visual(
            Cylinder(radius=0.096, length=0.036),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=wheel_rim,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.045, length=0.068),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=axle_steel,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.025, length=0.016),
            origin=Origin(
                xyz=(0.0, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=wheel_rim,
            name="cap",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=wheel_width),
            mass=1.5,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        model.articulation(
            f"body_to_{side_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x_pos, -0.31, 0.15)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=25.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.385, 0.915)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.025,
        max_penetration=0.0,
        positive_elem="lid_panel",
        negative_elem="shell",
        name="closed lid sits on the bin opening without sinking into the frame",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.52,
        elem_a="lid_panel",
        elem_b="shell",
        name="closed lid covers the full body opening footprint",
    )
    ctx.expect_origin_gap(
        right_wheel,
        left_wheel,
        axis="x",
        min_gap=0.58,
        max_gap=0.64,
        name="rear wheels are separated across the bin width",
    )
    ctx.expect_within(
        "left_wheel",
        "body",
        axes="yz",
        margin=0.19,
        name="left wheel axle center stays near the rear lower body region",
    )
    closed_front = ctx.part_element_world_aabb(lid, elem="front_grip")
    with ctx.pose({lid_hinge: 1.25}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_grip")
        ctx.check(
            "lid front edge rises when opened",
            closed_front is not None
            and open_front is not None
            and open_front[0][2] > closed_front[0][2] + 0.22,
            details=f"closed={closed_front}, open={open_front}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
