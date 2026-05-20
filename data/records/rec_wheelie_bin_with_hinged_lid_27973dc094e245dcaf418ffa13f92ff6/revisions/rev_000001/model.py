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
    rounded_rect_profile,
    ExtrudeGeometry,
)


TOP_Z = 0.700
HINGE_X = -0.220
HINGE_Z = 0.705
AXLE_X = -0.155
AXLE_Z = 0.120


def _loop(mesh: MeshGeometry, profile: list[tuple[float, float]], z: float) -> list[int]:
    return [mesh.add_vertex(x, y, z) for x, y in profile]


def _bridge(mesh: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    count = len(a)
    for index in range(count):
        a0 = a[index]
        a1 = a[(index + 1) % count]
        b0 = b[index]
        b1 = b[(index + 1) % count]
        if flip:
            mesh.add_face(a0, b1, a1)
            mesh.add_face(a0, b0, b1)
        else:
            mesh.add_face(a0, a1, b1)
            mesh.add_face(a0, b1, b0)


def _cap(mesh: MeshGeometry, loop: list[int], *, flip: bool = False) -> None:
    cx = sum(mesh.vertices[i][0] for i in loop) / len(loop)
    cy = sum(mesh.vertices[i][1] for i in loop) / len(loop)
    cz = sum(mesh.vertices[i][2] for i in loop) / len(loop)
    center = mesh.add_vertex(cx, cy, cz)
    for index in range(len(loop)):
        a = loop[index]
        b = loop[(index + 1) % len(loop)]
        if flip:
            mesh.add_face(center, b, a)
        else:
            mesh.add_face(center, a, b)


def _bin_shell_geometry() -> MeshGeometry:
    """Thin, open, tapered rectangular bin body with rounded molded corners."""
    wall = 0.020
    floor = 0.030
    sections = (
        (0.130, 0.300, 0.340),
        (0.410, 0.350, 0.410),
        (TOP_Z, 0.390, 0.470),
    )

    mesh = MeshGeometry()
    outer_loops: list[list[int]] = []
    inner_loops: list[list[int]] = []

    for z, depth, width in sections:
        outer_profile = rounded_rect_profile(depth, width, 0.045, corner_segments=8)
        outer_loops.append(_loop(mesh, outer_profile, z))

    for z, depth, width in (
        (0.130 + floor, 0.300 - 2.0 * wall, 0.340 - 2.0 * wall),
        (0.410, 0.350 - 2.0 * wall, 0.410 - 2.0 * wall),
        (TOP_Z, 0.390 - 2.0 * wall, 0.470 - 2.0 * wall),
    ):
        inner_profile = rounded_rect_profile(depth, width, 0.028, corner_segments=8)
        inner_loops.append(_loop(mesh, inner_profile, z))

    for lower, upper in zip(outer_loops, outer_loops[1:]):
        _bridge(mesh, lower, upper)
    for lower, upper in zip(inner_loops, inner_loops[1:]):
        _bridge(mesh, lower, upper, flip=True)

    _bridge(mesh, outer_loops[-1], inner_loops[-1], flip=True)
    _cap(mesh, outer_loops[0], flip=True)
    _cap(mesh, inner_loops[0])
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin")

    green = model.material("molded_green", rgba=(0.05, 0.34, 0.16, 1.0))
    dark_green = model.material("dark_green_detail", rgba=(0.025, 0.22, 0.10, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    grey = model.material("dark_plastic_hub", rgba=(0.08, 0.08, 0.075, 1.0))

    body = model.part("bin_body")
    body.visual(
        mesh_from_geometry(_bin_shell_geometry(), "bin_body_shell"),
        material=green,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.0153, length=0.610),
        origin=Origin(xyz=(AXLE_X, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="rear_axle",
    )
    for y in (-0.185, 0.185):
        body.visual(
            Box((0.075, 0.035, 0.070)),
            origin=Origin(xyz=(AXLE_X, y, AXLE_Z + 0.022)),
            material=dark_green,
            name=f"axle_boss_{0 if y < 0 else 1}",
        )
    body.visual(
        Box((0.018, 0.390, 0.055)),
        origin=Origin(xyz=(-0.205, 0.0, 0.660)),
        material=dark_green,
        name="rear_handle_bar",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.490),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="hinge_pin",
    )

    for y, length in ((-0.185, 0.080), (0.0, 0.080), (0.185, 0.080)):
        body.visual(
            Cylinder(radius=0.012, length=length),
            origin=Origin(xyz=(HINGE_X, y, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_green,
            name=f"body_hinge_knuckle_{0 if y < -0.01 else (1 if y < 0.01 else 2)}",
        )
        body.visual(
            Box((0.050, length, 0.014)),
            origin=Origin(xyz=(HINGE_X + 0.020, y, HINGE_Z - 0.015)),
            material=dark_green,
            name=f"body_hinge_leaf_{0 if y < -0.01 else (1 if y < 0.01 else 2)}",
        )

    lid = model.part("lid")
    lid_depth = 0.440
    lid_width = 0.520
    lid_thickness = 0.028
    lid.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(lid_depth, lid_width, 0.045, corner_segments=8),
                lid_thickness,
            ),
            "lid_panel",
        ),
        origin=Origin(xyz=(lid_depth / 2.0 + 0.025, 0.0, 0.003)),
        material=green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.038, 0.445, 0.030)),
        origin=Origin(xyz=(lid_depth + 0.030, 0.0, 0.010)),
        material=dark_green,
        name="front_grip_lip",
    )
    for index, y in enumerate((-0.135, 0.0, 0.135)):
        lid.visual(
            Box((0.300, 0.024, 0.014)),
            origin=Origin(xyz=(0.275, y, lid_thickness + 0.006)),
            material=dark_green,
            name=f"top_rib_{index}",
        )
    for index, y in enumerate((-0.095, 0.095)):
        lid.visual(
            Cylinder(radius=0.012, length=0.090),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_green,
            name=f"lid_hinge_knuckle_{index}",
        )
        lid.visual(
            Box((0.068, 0.090, 0.010)),
            origin=Origin(xyz=(0.046, y, -0.002)),
            material=dark_green,
            name=f"lid_hinge_leaf_{index}",
        )

    tire = TireGeometry(
        0.085,
        0.060,
        inner_radius=0.058,
        tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.58),
        sidewall=TireSidewall(style="square", bulge=0.02),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    rim = WheelGeometry(
        0.061,
        0.050,
        rim=WheelRim(inner_radius=0.036, flange_height=0.006, flange_thickness=0.004),
        hub=WheelHub(radius=0.024, width=0.046, cap_style="flat"),
        face=WheelFace(dish_depth=0.003, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.004, window_radius=0.010),
        bore=WheelBore(style="round", diameter=0.030),
    )
    tire_mesh = mesh_from_geometry(tire, "wheel_tire")
    rim_mesh = mesh_from_geometry(rim, "wheel_rim")

    for index, y in enumerate((-0.285, 0.285)):
        wheel = model.part(f"wheel_{index}")
        wheel.visual(tire_mesh, material=black, name="tire")
        wheel.visual(rim_mesh, material=grey, name="rim")
        model.articulation(
            f"body_to_wheel_{index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(AXLE_X, y, AXLE_Z), rpy=(0.0, 0.0, math.pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=12.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bin_body")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("body_to_lid")
    wheel_joints = [
        object_model.get_articulation("body_to_wheel_0"),
        object_model.get_articulation("body_to_wheel_1"),
    ]
    wheels = [object_model.get_part("wheel_0"), object_model.get_part("wheel_1")]

    for wheel in wheels:
        ctx.allow_overlap(
            body,
            wheel,
            elem_a="rear_axle",
            elem_b="rim",
            reason="The wheel hub is intentionally captured around the rear axle shaft so it can spin on the axle.",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="xyz",
            elem_a="rear_axle",
            elem_b="rim",
            min_overlap=0.020,
            name=f"{wheel.name} hub is retained on the rear axle",
        )
    for knuckle in ("lid_hinge_knuckle_0", "lid_hinge_knuckle_1"):
        ctx.allow_overlap(
            body,
            lid,
            elem_a="hinge_pin",
            elem_b=knuckle,
            reason="The lid hinge knuckle is intentionally captured by the hinge pin along the rear edge.",
        )
        ctx.expect_overlap(
            body,
            lid,
            axes="xyz",
            elem_a="hinge_pin",
            elem_b=knuckle,
            min_overlap=0.008,
            name=f"{knuckle} is captured by the hinge pin",
        )

    ctx.check(
        "rear wheels use continuous spin joints",
        all(joint.articulation_type == ArticulationType.CONTINUOUS for joint in wheel_joints),
    )
    ctx.check(
        "lid has realistic hinge limits",
        lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > 1.4,
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="body_shell",
            min_overlap=0.30,
            name="closed lid covers the full-width bin opening",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="body_shell",
            min_gap=0.0,
            max_gap=0.020,
            name="closed lid sits shallow above the top rim",
        )
        closed_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    with ctx.pose({lid_hinge: 1.25}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid opens upward about the rear edge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.20,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
