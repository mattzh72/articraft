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
)


BODY_HEIGHT = 0.266
CAP_BOTTOM_Z = 0.2400
CAP_HEIGHT = 0.031


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_ring(mesh: MeshGeometry, radius: float, z: float, segments: int):
    return [
        mesh.add_vertex(
            radius * math.cos((math.tau * index) / segments),
            radius * math.sin((math.tau * index) / segments),
            z,
        )
        for index in range(segments)
    ]


def _stitch_loops(mesh: MeshGeometry, loop_a, loop_b, *, flip: bool = False):
    count = len(loop_a)
    for index in range(count):
        next_index = (index + 1) % count
        a0 = loop_a[index]
        a1 = loop_a[next_index]
        b0 = loop_b[index]
        b1 = loop_b[next_index]
        if flip:
            mesh.add_face(a0, b1, a1)
            mesh.add_face(a0, b0, b1)
        else:
            mesh.add_face(a0, a1, b1)
            mesh.add_face(a0, b1, b0)


def _fill_disc(mesh: MeshGeometry, loop, z: float, *, upward: bool):
    center = mesh.add_vertex(0.0, 0.0, z)
    count = len(loop)
    for index in range(count):
        next_index = (index + 1) % count
        if upward:
            mesh.add_face(center, loop[index], loop[next_index])
        else:
            mesh.add_face(center, loop[next_index], loop[index])


def _fill_annulus(mesh: MeshGeometry, outer_loop, inner_loop, *, upward: bool):
    count = len(outer_loop)
    for index in range(count):
        next_index = (index + 1) % count
        o0 = outer_loop[index]
        o1 = outer_loop[next_index]
        i0 = inner_loop[index]
        i1 = inner_loop[next_index]
        if upward:
            mesh.add_face(i0, o0, o1)
            mesh.add_face(i0, o1, i1)
        else:
            mesh.add_face(i0, o1, o0)
            mesh.add_face(i0, i1, o1)


def _build_bottle_shell():
    outer_profile = [
        (0.0340, 0.0000),
        (0.0365, 0.0100),
        (0.0365, 0.0600),
        (0.0348, 0.1100),
        (0.0338, 0.1550),
        (0.0350, 0.1880),
        (0.0338, 0.2050),
        (0.0300, 0.2200),
        (0.0240, 0.2310),
        (0.0192, 0.2360),
        (0.0160, 0.2400),
        (0.0152, 0.2425),
        (0.0160, 0.2470),
        (0.0152, 0.2495),
        (0.0160, 0.2540),
        (0.0152, 0.2565),
        (0.0160, 0.2610),
        (0.0152, 0.2655),
    ]
    inner_profile = [
        (0.0000, 0.0000),
        (0.0100, 0.0030),
        (0.0310, 0.0100),
        (0.0310, 0.0600),
        (0.0296, 0.1100),
        (0.0288, 0.1550),
        (0.0300, 0.1880),
        (0.0290, 0.2040),
        (0.0250, 0.2180),
        (0.0190, 0.2300),
        (0.0110, 0.2430),
        (0.0110, 0.2655),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _build_cap_shell():
    outer_profile = [
        (0.0208, 0.0000),
        (0.0208, 0.0240),
        (0.0198, 0.0285),
        (0.0176, 0.0310),
    ]
    inner_profile = [
        (0.0182, 0.0000),
        (0.0178, 0.0040),
        (0.0178, 0.0242),
        (0.0145, 0.0286),
        (0.0000, 0.0310),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=84,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="screw_cap_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.76, 0.87, 0.94, 0.62))
    cap_plastic = model.material("cap_plastic", rgba=(0.94, 0.95, 0.97, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_body.visual(
        _mesh("bottle_shell", _build_bottle_shell()),
        material=bottle_plastic,
        name="bottle_shell",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=BODY_HEIGHT),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    cap = model.part("cap")
    cap.visual(
        _mesh("cap_shell", _build_cap_shell()),
        material=cap_plastic,
        name="cap_shell",
    )
    cap.visual(
        Cylinder(radius=0.0142, length=0.0055),
        origin=Origin(xyz=(0.0, 0.0, 0.02825)),
        material=cap_plastic,
        name="seal_liner",
    )
    rib_count = 20
    for index in range(rib_count):
        angle = (math.tau * index) / rib_count
        rib_radius = 0.0192
        cap.visual(
            Box((0.0036, 0.0058, 0.0220)),
            origin=Origin(
                xyz=(
                    rib_radius * math.cos(angle),
                    rib_radius * math.sin(angle),
                    CAP_HEIGHT * 0.50,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=cap_plastic,
            name=f"grip_rib_{index:02d}",
        )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=CAP_HEIGHT),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.0, CAP_HEIGHT * 0.5)),
    )

    model.articulation(
        "cap_spin",
        ArticulationType.CONTINUOUS,
        parent=bottle_body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle_body = object_model.get_part("bottle_body")
    cap = object_model.get_part("cap")
    cap_spin = object_model.get_articulation("cap_spin")

    ctx.expect_origin_distance(
        cap,
        bottle_body,
        axes="xy",
        max_dist=0.001,
        name="cap stays centered on the bottle axis",
    )
    ctx.expect_origin_gap(
        cap,
        bottle_body,
        axis="z",
        min_gap=0.235,
        max_gap=0.250,
        name="cap is mounted at the neck end of the bottle",
    )
    ctx.expect_overlap(
        cap,
        bottle_body,
        axes="xy",
        min_overlap=0.030,
        name="cap shares the bottle neck footprint",
    )
    ctx.expect_contact(
        cap,
        bottle_body,
        elem_a="seal_liner",
        elem_b="bottle_shell",
        name="cap liner seats on the bottle mouth",
    )

    bottle_aabb = ctx.part_world_aabb(bottle_body)
    cap_aabb = ctx.part_world_aabb(cap)
    bottle_height = None if bottle_aabb is None else bottle_aabb[1][2] - bottle_aabb[0][2]
    cap_height = None if cap_aabb is None else cap_aabb[1][2] - cap_aabb[0][2]
    ctx.check(
        "tall bottle with a compact closure",
        bottle_height is not None
        and cap_height is not None
        and bottle_height > 0.25
        and cap_height < 0.04
        and cap_height < bottle_height * 0.18,
        details=f"bottle_height={bottle_height}, cap_height={cap_height}",
    )

    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_spin: 1.7}):
        spun_pos = ctx.part_world_position(cap)
        ctx.expect_origin_distance(
            cap,
            bottle_body,
            axes="xy",
            max_dist=0.001,
            name="cap remains coaxial while spinning",
        )
    ctx.check(
        "cap rotates in place about the neck axis",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest_pos={rest_pos}, spun_pos={spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
