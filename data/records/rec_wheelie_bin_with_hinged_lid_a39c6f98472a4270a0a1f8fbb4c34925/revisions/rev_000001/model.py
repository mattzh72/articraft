from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
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
    rounded_rect_profile,
)


def _add_loop(mesh: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [mesh.add_vertex(x, y, z) for x, y, z in points]


def _bridge(mesh: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    count = len(a)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            mesh.add_face(a[i], b[j], a[j])
            mesh.add_face(a[i], b[i], b[j])
        else:
            mesh.add_face(a[i], a[j], b[j])
            mesh.add_face(a[i], b[j], b[i])


def _cap_loop(mesh: MeshGeometry, loop: list[int], center: tuple[float, float, float], *, flip: bool = False) -> None:
    c = mesh.add_vertex(*center)
    count = len(loop)
    for i in range(count):
        j = (i + 1) % count
        if flip:
            mesh.add_face(c, loop[j], loop[i])
        else:
            mesh.add_face(c, loop[i], loop[j])


def _rounded_loop(x_size: float, y_size: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(x_size, y_size, radius, corner_segments=7)]


def _tapered_bin_shell() -> MeshGeometry:
    """Open, hollow, tapered wheelie-bin bucket with rounded rectangular corners."""
    wall = 0.028
    floor_top = 0.205
    outer_sections = [
        (0.160, 0.480, 0.420, 0.050),
        (0.360, 0.555, 0.475, 0.055),
        (0.820, 0.675, 0.555, 0.066),
        (1.050, 0.720, 0.580, 0.070),
    ]
    mesh = MeshGeometry()

    outer_loops: list[list[int]] = []
    inner_loops: list[list[int]] = []
    for z, x_size, y_size, radius in outer_sections:
        outer_loops.append(_add_loop(mesh, _rounded_loop(x_size, y_size, radius, z)))

        inner_z = max(z, floor_top)
        inner_loops.append(
            _add_loop(
                mesh,
                _rounded_loop(
                    x_size - 2.0 * wall,
                    y_size - 2.0 * wall,
                    max(0.018, radius - wall),
                    inner_z,
                ),
            )
        )

    for i in range(len(outer_loops) - 1):
        _bridge(mesh, outer_loops[i], outer_loops[i + 1])

    # Reversed bridge makes the interior wall face into the hollow bin.
    for i in range(len(inner_loops) - 1):
        _bridge(mesh, inner_loops[i + 1], inner_loops[i], flip=True)

    # Top rolled rim and closed molded floor make the shell one connected body.
    _bridge(mesh, outer_loops[-1], inner_loops[-1], flip=True)
    _bridge(mesh, outer_loops[0], inner_loops[0])
    _cap_loop(mesh, outer_loops[0], (0.0, 0.0, outer_sections[0][0]), flip=True)
    _cap_loop(mesh, inner_loops[0], (0.0, 0.0, floor_top))
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelie_bin_with_lid")

    bin_green = Material("molded_bin_green", color=(0.05, 0.34, 0.16, 1.0))
    dark_green = Material("dark_green_edges", color=(0.025, 0.18, 0.08, 1.0))
    black_rubber = Material("black_rubber", color=(0.01, 0.01, 0.009, 1.0))
    axle_gray = Material("dark_axle_gray", color=(0.08, 0.085, 0.085, 1.0))
    hub_gray = Material("recycled_plastic_gray", color=(0.22, 0.23, 0.22, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_tapered_bin_shell(), "tapered_bin_shell"),
        material=bin_green,
        name="body_shell",
    )

    stop_outer = rounded_rect_profile(0.760, 0.620, 0.075, corner_segments=7)
    stop_inner = rounded_rect_profile(0.635, 0.495, 0.050, corner_segments=7)
    body.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(stop_outer, [stop_inner], 0.0425, cap=True, center=True),
            "stop_flange",
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.07125)),
        material=dark_green,
        name="stop_flange",
    )

    # Rear axle and molded pockets tie the wheel joints visibly back into the bin.
    body.visual(
        Cylinder(radius=0.016, length=0.750),
        origin=Origin(xyz=(-0.280, 0.0, 0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=axle_gray,
        name="rear_axle",
    )
    for idx, y in enumerate((-0.2885, 0.2885)):
        body.visual(
            Cylinder(radius=0.052, length=0.013),
            origin=Origin(xyz=(-0.280, y, 0.120), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=axle_gray,
            name=f"wheel_retainer_{idx}",
        )
    for idx, y in enumerate((-0.235, 0.235)):
        body.visual(
            Box((0.135, 0.085, 0.135)),
            origin=Origin(xyz=(-0.285, y, 0.135)),
            material=dark_green,
            name=f"axle_pocket_{idx}",
        )

    # Small front feet keep the non-wheel end grounded like a real wheelie bin.
    for idx, y in enumerate((-0.155, 0.155)):
        body.visual(
            Box((0.090, 0.060, 0.180)),
            origin=Origin(xyz=(0.205, y, 0.090)),
            material=dark_green,
            name=f"front_foot_{idx}",
        )

    # Alternating fixed hinge barrels at the rear lip.
    for name, y, length in (
        ("rear_hinge_center", 0.0, 0.155),
        ("rear_hinge_end_0", -0.292, 0.072),
        ("rear_hinge_end_1", 0.292, 0.072),
    ):
        body.visual(
            Cylinder(radius=0.022, length=length),
            origin=Origin(xyz=(-0.360, y, 1.115), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_green,
            name=name,
        )
        body.visual(
            Box((0.050, length, 0.058)),
            origin=Origin(xyz=(-0.340, y, 1.083)),
            material=dark_green,
            name=f"{name}_strap",
        )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.820, 0.660, 0.075, corner_segments=7), 0.045, cap=True, center=True),
            "lid_panel",
        ),
        origin=Origin(xyz=(0.455, 0.0, 0.0)),
        material=bin_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.042, 0.570, 0.070)),
        origin=Origin(xyz=(0.840, 0.0, -0.020)),
        material=dark_green,
        name="front_lid_skirt",
    )
    for idx, y in enumerate((-0.205, 0.0, 0.205)):
        lid.visual(
            Box((0.530, 0.025, 0.018)),
            origin=Origin(xyz=(0.500, y, 0.029)),
            material=dark_green,
            name=f"lid_rib_{idx}",
        )
    for idx, y in enumerate((-0.160, 0.160)):
        lid.visual(
            Cylinder(radius=0.021, length=0.130),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_green,
            name=f"lid_hinge_barrel_{idx}",
        )
        lid.visual(
            Box((0.090, 0.130, 0.020)),
            origin=Origin(xyz=(0.045, y, -0.006)),
            material=dark_green,
            name=f"lid_hinge_strap_{idx}",
        )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.120,
            0.078,
            inner_radius=0.086,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.007, radius=0.003),
        ),
        "wheel_tire",
    )
    hub_mesh = mesh_from_geometry(
        WheelGeometry(
            0.085,
            0.080,
            rim=WheelRim(inner_radius=0.050, flange_height=0.007, flange_thickness=0.004),
            hub=WheelHub(radius=0.038, width=0.076, cap_style="flat"),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.006, window_radius=0.012),
            bore=WheelBore(style="round", diameter=0.046),
        ),
        "wheel_hub",
    )

    for idx, y in enumerate((-0.335, 0.335)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black_rubber,
            name="tire",
        )
        wheel.visual(
            hub_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=hub_gray,
            name="hub",
        )
        model.articulation(
            f"body_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(-0.280, y, 0.120)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=18.0),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.360, 0.0, 1.115)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    lid_hinge = object_model.get_articulation("body_to_lid")
    wheel_joint_0 = object_model.get_articulation("body_to_wheel_0")
    wheel_joint_1 = object_model.get_articulation("body_to_wheel_1")

    ctx.check(
        "lid hinge is revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lid_hinge.articulation_type}",
    )
    ctx.check(
        "wheels spin continuously",
        wheel_joint_0.articulation_type == ArticulationType.CONTINUOUS
        and wheel_joint_1.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={wheel_joint_0.articulation_type}, {wheel_joint_1.articulation_type}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="stop_flange",
            max_gap=0.001,
            max_penetration=0.0005,
            name="closed lid seats on stop flange",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="stop_flange",
            min_overlap=0.45,
            name="full width lid covers the opening lip",
        )
        closed_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "positive lid motion opens upward",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.35,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for idx, wheel in enumerate((wheel_0, wheel_1)):
        ctx.expect_overlap(
            wheel,
            body,
            axes="xz",
            elem_a="hub",
            elem_b="rear_axle",
            min_overlap=0.025,
            name=f"wheel_{idx} hub is centered on rear axle",
        )

    return ctx.report()


object_model = build_object_model()
