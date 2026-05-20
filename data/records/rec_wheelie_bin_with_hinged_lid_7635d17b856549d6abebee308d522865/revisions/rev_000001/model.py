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
)


def _rounded_rect_loop(depth: float, width: float, radius: float, segments: int = 5):
    """Return an XY loop for a rounded rectangle, centered on the origin.

    The loop uses X for bin depth and Y for bin width.
    """

    hx = depth * 0.5
    hy = width * 0.5
    r = min(radius, hx * 0.45, hy * 0.45)
    centers = (
        (hx - r, hy - r, 0.0, math.pi * 0.5),
        (-hx + r, hy - r, math.pi * 0.5, math.pi),
        (-hx + r, -hy + r, math.pi, math.pi * 1.5),
        (hx - r, -hy + r, math.pi * 1.5, math.pi * 2.0),
    )
    pts = []
    for cx, cy, a0, a1 in centers:
        for i in range(segments + 1):
            if pts and i == 0:
                continue
            a = a0 + (a1 - a0) * i / segments
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _add_loop(mesh: MeshGeometry, loop, z: float) -> list[int]:
    return [mesh.add_vertex(x, y, z) for x, y in loop]


def _connect_loops(mesh: MeshGeometry, lower: list[int], upper: list[int], *, flip: bool = False) -> None:
    n = len(lower)
    for i in range(n):
        j = (i + 1) % n
        if not flip:
            mesh.add_face(lower[i], lower[j], upper[j])
            mesh.add_face(lower[i], upper[j], upper[i])
        else:
            mesh.add_face(lower[i], upper[j], lower[j])
            mesh.add_face(lower[i], upper[i], upper[j])


def _make_bin_shell_mesh() -> MeshGeometry:
    """Thin-walled tapered wheelie-bin tub, open at the top and closed at the floor."""

    outer_bottom = _rounded_rect_loop(0.44, 0.42, 0.055)
    outer_top = _rounded_rect_loop(0.66, 0.56, 0.075)
    inner_floor = _rounded_rect_loop(0.36, 0.34, 0.045)
    inner_top = _rounded_rect_loop(0.59, 0.49, 0.055)

    mesh = MeshGeometry()
    ob = _add_loop(mesh, outer_bottom, 0.08)
    ot = _add_loop(mesh, outer_top, 0.94)
    it = _add_loop(mesh, inner_top, 0.91)
    ib = _add_loop(mesh, inner_floor, 0.16)

    # Outer side wall.
    _connect_loops(mesh, ob, ot)
    # Inner visible cavity wall, with reversed orientation.
    _connect_loops(mesh, ib, it, flip=True)
    # Rolled top rim: connects the outside wall to the inner cavity wall.
    _connect_loops(mesh, it, ot)
    # Closed molded floor with a recessed inside bottom.
    _connect_loops(mesh, ob, ib, flip=True)
    center = mesh.add_vertex(0.0, 0.0, 0.16)
    for i in range(len(ib)):
        mesh.add_face(center, ib[i], ib[(i + 1) % len(ib)])

    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_wheelie_bin")

    green = model.material("recycled_green_hdpe", rgba=(0.08, 0.38, 0.18, 1.0))
    dark_green = model.material("shadowed_green_hdpe", rgba=(0.045, 0.22, 0.105, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    grey = model.material("galvanized_axle", rgba=(0.55, 0.56, 0.54, 1.0))
    wheel_grey = model.material("molded_grey_wheel", rgba=(0.20, 0.22, 0.22, 1.0))

    shell = model.part("bin_shell")
    shell.visual(
        mesh_from_geometry(_make_bin_shell_mesh(), "tapered_open_bin_shell"),
        material=green,
        name="shell_tub",
    )

    # A few connected molded ribs and one-piece rims keep the tub stiff while
    # staying cheap to mold.
    shell.visual(
        Box((0.12, 0.42, 0.62)),
        origin=Origin(xyz=(0.285, 0.0, 0.48)),
        material=dark_green,
        name="front_stiffener",
    )
    shell.visual(
        Box((0.070, 0.10, 0.72)),
        origin=Origin(xyz=(-0.34, 0.205, 0.50)),
        material=dark_green,
        name="rear_stay_0",
    )
    shell.visual(
        Box((0.070, 0.10, 0.72)),
        origin=Origin(xyz=(-0.34, -0.205, 0.50)),
        material=dark_green,
        name="rear_stay_1",
    )

    # Full-width rear handle molded into the bin: no separate fasteners.
    shell.visual(
        Cylinder(radius=0.018, length=0.52),
        origin=Origin(xyz=(-0.405, 0.0, 0.82), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=green,
        name="rear_handle",
    )
    shell.visual(
        Box((0.12, 0.050, 0.060)),
        origin=Origin(xyz=(-0.36, 0.245, 0.82)),
        material=green,
        name="handle_support_0",
    )
    shell.visual(
        Box((0.12, 0.050, 0.060)),
        origin=Origin(xyz=(-0.36, -0.245, 0.82)),
        material=green,
        name="handle_support_1",
    )

    # Molded rear axle saddle and a simple steel through-rod.  The saddle shows
    # the assembly order: rod slides through the bin, then wheels snap on.
    shell.visual(
        Box((0.13, 0.54, 0.075)),
        origin=Origin(xyz=(-0.305, 0.0, 0.145)),
        material=dark_green,
        name="axle_saddle",
    )
    shell.visual(
        Cylinder(radius=0.013, length=0.78),
        origin=Origin(xyz=(-0.315, 0.0, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grey,
        name="axle",
    )
    for y, suffix in ((0.305, "0"), (-0.305, "1")):
        shell.visual(
            Box((0.075, 0.045, 0.11)),
            origin=Origin(xyz=(-0.315, y, 0.145)),
            material=dark_green,
            name=f"axle_boss_{suffix}",
        )
    for y, suffix in ((0.337, "0"), (-0.337, "1")):
        shell.visual(
            Cylinder(radius=0.041, length=0.006),
            origin=Origin(xyz=(-0.315, y, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=grey,
            name=f"wheel_retainer_{suffix}",
        )

    # A low-cost snap hinge: molded bin-side support ears plus one rod.
    shell.visual(
        Cylinder(radius=0.011, length=0.64),
        origin=Origin(xyz=(-0.39, 0.0, 0.96), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grey,
        name="hinge_pin",
    )
    for y, suffix in ((0.105, "0"), (-0.105, "1")):
        shell.visual(
            Box((0.075, 0.036, 0.060)),
            origin=Origin(xyz=(-0.36, y, 0.935)),
            material=green,
            name=f"hinge_ear_{suffix}",
        )

    lid = model.part("lid")
    lid.visual(
        Box((0.75, 0.64, 0.030)),
        origin=Origin(xyz=(0.375, 0.0, 0.028)),
        material=green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.030, 0.60, 0.080)),
        origin=Origin(xyz=(0.735, 0.0, -0.010)),
        material=green,
        name="front_apron",
    )
    lid.visual(
        Box((0.72, 0.028, 0.060)),
        origin=Origin(xyz=(0.38, 0.314, 0.000)),
        material=green,
        name="side_lip_0",
    )
    lid.visual(
        Box((0.72, 0.028, 0.060)),
        origin=Origin(xyz=(0.38, -0.314, 0.000)),
        material=green,
        name="side_lip_1",
    )
    for y, suffix in ((0.0, "0"), (0.205, "1"), (-0.205, "2")):
        lid.visual(
            Cylinder(radius=0.023, length=0.120),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=green,
            name=f"hinge_barrel_{suffix}",
        )
        lid.visual(
            Box((0.095, 0.105, 0.028)),
            origin=Origin(xyz=(0.070, y, 0.013)),
            material=green,
            name=f"barrel_strap_{suffix}",
        )
    lid.visual(
        Box((0.55, 0.035, 0.018)),
        origin=Origin(xyz=(0.41, 0.16, 0.052)),
        material=dark_green,
        name="lid_rib_0",
    )
    lid.visual(
        Box((0.55, 0.035, 0.018)),
        origin=Origin(xyz=(0.41, -0.16, 0.052)),
        material=dark_green,
        name="lid_rib_1",
    )

    model.articulation(
        "shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lid,
        origin=Origin(xyz=(-0.39, 0.0, 0.96)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.85),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.115,
            0.066,
            inner_radius=0.083,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.004, count=18, land_ratio=0.62),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="square", bulge=0.02),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "bin_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.085,
            0.056,
            rim=WheelRim(inner_radius=0.055, flange_height=0.006, flange_thickness=0.004),
            hub=WheelHub(
                radius=0.030,
                width=0.052,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.038, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.005, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.030),
        ),
        "bin_wheel",
    )

    for y, suffix in ((0.365, "0"), (-0.365, "1")):
        wheel = model.part(f"wheel_{suffix}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=wheel_grey,
            name="rim",
        )
        model.articulation(
            f"shell_to_wheel_{suffix}",
            ArticulationType.CONTINUOUS,
            parent=shell,
            child=wheel,
            origin=Origin(xyz=(-0.315, y, 0.145)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=12.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("bin_shell")
    lid = object_model.get_part("lid")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    lid_hinge = object_model.get_articulation("shell_to_lid")
    wheel_joint_0 = object_model.get_articulation("shell_to_wheel_0")

    for barrel_name in ("hinge_barrel_0", "hinge_barrel_1", "hinge_barrel_2"):
        ctx.allow_overlap(
            shell,
            lid,
            elem_a="hinge_pin",
            elem_b=barrel_name,
            reason="The steel hinge rod is intentionally captured inside the molded lid barrel.",
        )
        ctx.expect_within(
            shell,
            lid,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem=barrel_name,
            margin=0.002,
            name=f"{barrel_name} captures hinge rod diameter",
        )
        ctx.expect_overlap(
            shell,
            lid,
            axes="y",
            elem_a="hinge_pin",
            elem_b=barrel_name,
            min_overlap=0.06,
            name=f"{barrel_name} is retained on the hinge rod",
        )

    for wheel, retainer, positive_wheel in (
        (wheel_0, "wheel_retainer_0", True),
        (wheel_1, "wheel_retainer_1", False),
    ):
        ctx.allow_overlap(
            shell,
            wheel,
            elem_a="axle",
            elem_b="rim",
            reason="The wheel hub is represented as a captured plastic bore around the steel axle.",
        )
        ctx.allow_overlap(
            shell,
            wheel,
            elem_a=retainer,
            elem_b="rim",
            reason="A snap-on axle retainer washer lightly captures the wheel hub side face.",
        )
        ctx.expect_within(
            shell,
            wheel,
            axes="xz",
            inner_elem="axle",
            outer_elem="rim",
            margin=0.005,
            name=f"{wheel.name} hub surrounds axle",
        )
        ctx.expect_overlap(
            shell,
            wheel,
            axes="y",
            elem_a="axle",
            elem_b="rim",
            min_overlap=0.04,
            name=f"{wheel.name} is retained on axle",
        )
        if positive_wheel:
            ctx.expect_gap(
                wheel,
                shell,
                axis="y",
                positive_elem="rim",
                negative_elem=retainer,
                max_gap=0.001,
                max_penetration=0.004,
                name=f"{wheel.name} retainer seats against inner hub face",
            )
        else:
            ctx.expect_gap(
                shell,
                wheel,
                axis="y",
                positive_elem=retainer,
                negative_elem="rim",
                max_gap=0.001,
                max_penetration=0.004,
                name=f"{wheel.name} retainer seats against inner hub face",
            )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            shell,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="shell_tub",
            min_gap=0.005,
            max_gap=0.05,
            name="closed lid sits just above the rolled rim",
        )
        ctx.expect_overlap(
            lid,
            shell,
            axes="xy",
            elem_a="lid_panel",
            elem_b="shell_tub",
            min_overlap=0.45,
            name="closed lid covers the bin opening",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.2}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid hinge swings upward",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.25,
        details=f"closed={closed_aabb}, opened={open_aabb}",
    )

    wheel_rest = ctx.part_world_aabb(wheel_0)
    with ctx.pose({wheel_joint_0: math.pi / 2.0}):
        wheel_rotated = ctx.part_world_aabb(wheel_0)
    ctx.check(
        "wheel joint supports rolling rotation",
        wheel_rest is not None and wheel_rotated is not None,
        details=f"rest={wheel_rest}, rotated={wheel_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
