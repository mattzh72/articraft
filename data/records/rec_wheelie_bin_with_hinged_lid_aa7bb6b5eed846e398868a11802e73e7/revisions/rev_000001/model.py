from __future__ import annotations

import math

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
    TireShoulder,
    TireSidewall,
    TireTread,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _clean_loop(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Remove a duplicated closing point if a profile helper returns one."""
    if len(points) > 1 and points[0] == points[-1]:
        return points[:-1]
    return points


def _rounded_loop(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    y_center: float = 0.0,
    segments: int = 7,
) -> list[tuple[float, float, float]]:
    profile = _clean_loop(
        rounded_rect_profile(width, depth, radius, corner_segments=segments)
    )
    return [(x, y + y_center, z) for x, y in profile]


def _add_loop(mesh: MeshGeometry, loop: list[tuple[float, float, float]]) -> list[int]:
    return [mesh.add_vertex(x, y, z) for x, y, z in loop]


def _bridge(mesh: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            mesh.add_face(a[i], b[j], b[i])
            mesh.add_face(a[i], a[j], b[j])
        else:
            mesh.add_face(a[i], b[i], b[j])
            mesh.add_face(a[i], b[j], a[j])


def _cap(mesh: MeshGeometry, loop: list[int], *, z: float, flip: bool = False) -> None:
    xs = [mesh.vertices[i][0] for i in loop]
    ys = [mesh.vertices[i][1] for i in loop]
    center = mesh.add_vertex(sum(xs) / len(xs), sum(ys) / len(ys), z)
    count = len(loop)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            mesh.add_face(center, loop[j], loop[i])
        else:
            mesh.add_face(center, loop[i], loop[j])


def _build_tapered_bin_body() -> MeshGeometry:
    """Thin-walled, open-topped wheelie-bin tub with a molded top rim."""
    mesh = MeshGeometry()

    outer_lower = _add_loop(mesh, _rounded_loop(0.44, 0.50, 0.060, 0.205, y_center=0.035))
    outer_mid = _add_loop(mesh, _rounded_loop(0.54, 0.66, 0.075, 0.620, y_center=0.035))
    outer_top = _add_loop(mesh, _rounded_loop(0.63, 0.80, 0.085, 0.950, y_center=0.035))
    inner_top = _add_loop(mesh, _rounded_loop(0.55, 0.72, 0.062, 0.928, y_center=0.035))
    inner_mid = _add_loop(mesh, _rounded_loop(0.47, 0.58, 0.055, 0.610, y_center=0.035))
    inner_floor = _add_loop(mesh, _rounded_loop(0.35, 0.40, 0.045, 0.290, y_center=0.035))

    _bridge(mesh, outer_lower, outer_mid)
    _bridge(mesh, outer_mid, outer_top)
    # The horizontal/rolled top rim reveals real wall thickness around the opening.
    _bridge(mesh, outer_top, inner_top, flip=True)
    _bridge(mesh, inner_top, inner_mid, flip=True)
    _bridge(mesh, inner_mid, inner_floor, flip=True)
    # Sloped closed bottom and inside floor keep the bin hollow but watertight.
    _bridge(mesh, inner_floor, outer_lower, flip=True)
    _cap(mesh, inner_floor, z=0.290, flip=True)

    return mesh


def _build_lid_cap() -> MeshGeometry:
    """A shallow, deep, rounded top lid whose local origin lies on the rear hinge line."""
    mesh = MeshGeometry()
    lower = _add_loop(mesh, _rounded_loop(0.69, 0.84, 0.080, 0.000, y_center=0.420))
    crown = _add_loop(mesh, _rounded_loop(0.65, 0.80, 0.070, 0.046, y_center=0.420))
    _bridge(mesh, lower, crown)
    _cap(mesh, crown, z=0.046)
    _cap(mesh, lower, z=0.000, flip=True)
    return mesh


def _add_wheel_visuals(part, *, tire_material, rim_material, hub_material, prefix: str) -> None:
    tire = TireGeometry(
        0.145,
        0.076,
        inner_radius=0.092,
        tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
        sidewall=TireSidewall(style="square", bulge=0.025),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    part.visual(mesh_from_geometry(tire, f"{prefix}_tire"), material=tire_material, name="tire")
    part.visual(
        Cylinder(radius=0.095, length=0.072),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rim_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.036, length=0.090),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hub_material,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    green_plastic = model.material("green_plastic", rgba=(0.08, 0.30, 0.18, 1.0))
    lid_green = model.material("lid_green", rgba=(0.10, 0.36, 0.20, 1.0))
    dark_green = model.material("dark_green", rgba=(0.045, 0.17, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.018, 0.018, 0.016, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.22, 1.0))
    grey_plastic = model.material("grey_plastic", rgba=(0.30, 0.32, 0.30, 1.0))
    cavity_shadow = model.material("cavity_shadow", rgba=(0.015, 0.030, 0.020, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_build_tapered_bin_body(), "tapered_bin_body"),
        material=green_plastic,
        name="body_shell",
    )
    body.visual(
        Box((0.35, 0.40, 0.006)),
        origin=Origin(xyz=(0.0, 0.035, 0.294)),
        material=cavity_shadow,
        name="inside_floor",
    )

    # Molded front ribs and side pads make the tub read as a plastic wheelie bin
    # rather than a simple tapered box.
    for x in (-0.18, 0.18):
        body.visual(
            Box((0.045, 0.030, 0.46)),
            origin=Origin(xyz=(x, 0.370, 0.565)),
            material=dark_green,
            name=f"front_rib_{0 if x < 0 else 1}",
        )
    body.visual(
        Box((0.44, 0.028, 0.055)),
        origin=Origin(xyz=(0.0, 0.397, 0.815)),
        material=dark_green,
        name="front_band",
    )
    for x, name in ((-0.272, "side_pad_0"), (0.272, "side_pad_1")):
        body.visual(
            Box((0.035, 0.34, 0.23)),
            origin=Origin(xyz=(x, 0.020, 0.570)),
            material=dark_green,
            name=name,
        )

    # Rear pull handle, hinge pads, and axle hardware are molded/mounted to the tub.
    body.visual(
        Cylinder(radius=0.020, length=0.66),
        origin=Origin(xyz=(0.0, -0.432, 0.860), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_green,
        name="rear_handle",
    )
    for x, name in ((-0.235, "handle_mount_0"), (0.235, "handle_mount_1")):
        body.visual(
            Box((0.085, 0.105, 0.070)),
            origin=Origin(xyz=(x, -0.389, 0.850)),
            material=green_plastic,
            name=name,
        )
    for x, name in ((-0.270, "hinge_pad_0"), (0.270, "hinge_pad_1")):
        body.visual(
            Box((0.115, 0.058, 0.060)),
            origin=Origin(xyz=(x, -0.369, 0.949)),
            material=green_plastic,
            name=name,
        )

    axle_y = -0.270
    axle_z = 0.145
    body.visual(
        Cylinder(radius=0.020, length=0.640),
        origin=Origin(xyz=(0.0, axle_y, axle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="rear_axle",
    )
    for x, name in ((-0.313, "axle_washer_0"), (0.313, "axle_washer_1")):
        body.visual(
            Cylinder(radius=0.052, length=0.014),
            origin=Origin(xyz=(x, axle_y, axle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=name,
        )
    for x, name in ((-0.235, "axle_bracket_0"), (0.235, "axle_bracket_1")):
        body.visual(
            Box((0.072, 0.100, 0.120)),
            origin=Origin(xyz=(x, -0.246, 0.185)),
            material=dark_green,
            name=name,
        )
    body.visual(
        Box((0.36, 0.070, 0.050)),
        origin=Origin(xyz=(0.0, -0.250, 0.245)),
        material=dark_green,
        name="rear_foot_bar",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_cap(), "short_deep_lid"),
        material=lid_green,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.660),
        origin=Origin(xyz=(0.0, -0.005, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lid_green,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.610, 0.040, 0.052)),
        origin=Origin(xyz=(0.0, 0.805, -0.004)),
        material=dark_green,
        name="front_lip",
    )
    for x, name in ((-0.255, "side_lip_0"), (0.255, "side_lip_1")):
        lid.visual(
            Box((0.044, 0.650, 0.030)),
            origin=Origin(xyz=(x, 0.435, -0.002)),
            material=dark_green,
            name=name,
        )

    wheel_0 = model.part("wheel_0")
    _add_wheel_visuals(
        wheel_0,
        tire_material=black_rubber,
        rim_material=grey_plastic,
        hub_material=dark_steel,
        prefix="wheel_0",
    )
    wheel_1 = model.part("wheel_1")
    _add_wheel_visuals(
        wheel_1,
        tire_material=black_rubber,
        rim_material=grey_plastic,
        hub_material=dark_steel,
        prefix="wheel_1",
    )

    hinge_y = -0.385
    hinge_z = 0.985
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        # The closed lid extends forward along local +Y from the rear hinge.
        # +X makes positive q lift the front edge upward.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel_0,
        origin=Origin(xyz=(-0.365, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )
    model.articulation(
        "wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel_1,
        origin=Origin(xyz=(0.365, axle_y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lid = object_model.get_part("lid")
    body = object_model.get_part("body")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    hinge = object_model.get_articulation("lid_hinge")
    wheel_0_spin = object_model.get_articulation("wheel_0_spin")
    wheel_1_spin = object_model.get_articulation("wheel_1_spin")

    ctx.check(
        "lid uses rear horizontal revolute hinge",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (1.0, 0.0, 0.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper > 1.5,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge.motion_limits}",
    )
    for joint in (wheel_0_spin, wheel_1_spin):
        ctx.check(
            f"{joint.name} is continuous on the rear axle",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.axis == (1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            min_gap=0.0,
            max_gap=0.040,
            name="closed lid sits just above bin rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.45,
            name="full width lid covers the top opening",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({hinge: math.radians(95.0)}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "positive hinge angle lifts lid front edge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.45,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    wheel_0_pos = ctx.part_world_position(wheel_0)
    wheel_1_pos = ctx.part_world_position(wheel_1)
    ctx.check(
        "two wheels share one rear axle line",
        wheel_0_pos is not None
        and wheel_1_pos is not None
        and abs(wheel_0_pos[1] - wheel_1_pos[1]) < 1e-6
        and abs(wheel_0_pos[2] - wheel_1_pos[2]) < 1e-6
        and wheel_0_pos[0] < -0.30
        and wheel_1_pos[0] > 0.30,
        details=f"wheel_0={wheel_0_pos}, wheel_1={wheel_1_pos}",
    )

    return ctx.report()


object_model = build_object_model()
