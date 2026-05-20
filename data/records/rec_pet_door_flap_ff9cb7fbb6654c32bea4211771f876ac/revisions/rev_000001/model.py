from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_flap_pet_door")

    plastic = Material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    tunnel = Material("slightly_shadowed_tunnel", rgba=(0.68, 0.69, 0.66, 1.0))
    smoked_vinyl = Material("smoked_flexible_vinyl", rgba=(0.38, 0.47, 0.52, 0.58))
    amber_vinyl = Material("warm_clear_vinyl", rgba=(0.65, 0.55, 0.40, 0.55))
    dark_seal = Material("black_magnetic_seal", rgba=(0.03, 0.03, 0.028, 1.0))
    hinge_metal = Material("brushed_hinge_pin", rgba=(0.58, 0.58, 0.55, 1.0))

    frame = model.part("frame")

    # Two proud rounded rings sit on the wall faces; the four box liners between
    # them leave a real open tunnel rather than a solid proxy.
    bezel_profile = rounded_rect_profile(0.54, 0.70, 0.045, corner_segments=8)
    bezel_opening = rounded_rect_profile(0.39, 0.52, 0.020, corner_segments=8)
    bezel_mesh = ExtrudeWithHolesGeometry(
        bezel_profile,
        [bezel_opening],
        0.035,
        center=True,
    )
    for name, y in (("front_bezel", -0.1575), ("rear_bezel", 0.1575)):
        frame.visual(
            mesh_from_geometry(bezel_mesh, name),
            origin=Origin(xyz=(0.0, y, 0.35), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=plastic,
            name=name,
        )

    # Deep rectangular tunnel: clear passage is approximately 0.36 m by 0.50 m.
    frame.visual(
        Box((0.025, 0.282, 0.50)),
        origin=Origin(xyz=(-0.1925, 0.0, 0.34)),
        material=tunnel,
        name="left_tunnel_wall",
    )
    frame.visual(
        Box((0.025, 0.282, 0.50)),
        origin=Origin(xyz=(0.1925, 0.0, 0.34)),
        material=tunnel,
        name="right_tunnel_wall",
    )
    frame.visual(
        Box((0.410, 0.282, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.6025)),
        material=tunnel,
        name="top_tunnel_wall",
    )
    frame.visual(
        Box((0.410, 0.282, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=tunnel,
        name="bottom_threshold",
    )

    # Separate top bearing edges, one near each face of the tunnel.
    hinge_z = 0.545
    outer_y = -0.118
    inner_y = 0.118
    for prefix, y in (("outer", outer_y), ("inner", inner_y)):
        frame.visual(
            Box((0.360, 0.020, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.582)),
            material=dark_seal,
            name=f"{prefix}_top_edge",
        )
        for side, x in (("left", -0.1725), ("right", 0.1725)):
            frame.visual(
                Box((0.025, 0.040, 0.040)),
                origin=Origin(xyz=(x, y, hinge_z)),
                material=plastic,
                name=f"{prefix}_{side}_bearing",
            )

    def add_flap(
        part_name: str,
        joint_name: str,
        hinge_y: float,
        panel_material: Material,
    ) -> None:
        flap = model.part(part_name)
        flap_width = 0.330
        flap_height = 0.430

        # Child part frame is exactly on the upper hinge axis; all visible flap
        # geometry hangs downward from it at q=0.
        flap.visual(
            Box((flap_width, 0.006, flap_height)),
            origin=Origin(xyz=(0.0, 0.0, -0.226)),
            material=panel_material,
            name="flexible_panel",
        )
        flap.visual(
            Box((0.314, 0.014, 0.024)),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=dark_seal,
            name="top_clamp",
        )
        flap.visual(
            Cylinder(radius=0.009, length=0.315),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name="hinge_barrel",
        )
        flap.visual(
            Box((0.314, 0.014, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.431)),
            material=dark_seal,
            name="bottom_magnet",
        )
        flap.visual(
            Box((0.010, 0.010, 0.365)),
            origin=Origin(xyz=(-0.158, 0.0, -0.238)),
            material=dark_seal,
            name="side_seal_0",
        )
        flap.visual(
            Box((0.010, 0.010, 0.365)),
            origin=Origin(xyz=(0.158, 0.0, -0.238)),
            material=dark_seal,
            name="side_seal_1",
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=flap,
            origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-1.15,
                upper=1.15,
                effort=2.0,
                velocity=3.0,
            ),
        )

    add_flap("outer_flap", "outer_flap_hinge", outer_y, smoked_vinyl)
    add_flap("inner_flap", "inner_flap_hinge", inner_y, amber_vinyl)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    outer = object_model.get_part("outer_flap")
    inner = object_model.get_part("inner_flap")
    outer_hinge = object_model.get_articulation("outer_flap_hinge")
    inner_hinge = object_model.get_articulation("inner_flap_hinge")

    ctx.check(
        "two independent top hinges",
        (
            outer_hinge.articulation_type == ArticulationType.REVOLUTE
            and inner_hinge.articulation_type == ArticulationType.REVOLUTE
            and getattr(outer_hinge.child, "name", outer_hinge.child) == outer.name
            and getattr(inner_hinge.child, "name", inner_hinge.child) == inner.name
            and getattr(outer_hinge.parent, "name", outer_hinge.parent) == frame.name
            and getattr(inner_hinge.parent, "name", inner_hinge.parent) == frame.name
        ),
        details="Each flap should be a separate child of the frame on its own revolute hinge.",
    )
    ctx.check(
        "hinges span opposite tunnel faces",
        outer_hinge.origin.xyz[1] < -0.09 and inner_hinge.origin.xyz[1] > 0.09,
        details=f"outer_y={outer_hinge.origin.xyz[1]:.3f}, inner_y={inner_hinge.origin.xyz[1]:.3f}",
    )
    ctx.check(
        "hinge axes are horizontal",
        outer_hinge.axis == (1.0, 0.0, 0.0) and inner_hinge.axis == (1.0, 0.0, 0.0),
        details=f"outer_axis={outer_hinge.axis}, inner_axis={inner_hinge.axis}",
    )
    ctx.expect_origin_gap(
        inner,
        outer,
        axis="y",
        min_gap=0.22,
        name="closed flaps are separated by the deep tunnel",
    )

    def panel_inside_tunnel(part_name: str) -> bool:
        aabb = ctx.part_element_world_aabb(part_name, elem="flexible_panel")
        if aabb is None:
            return False
        lo, hi = aabb
        return lo[0] > -0.171 and hi[0] < 0.171 and lo[2] > 0.09 and hi[2] < 0.54

    ctx.check(
        "outer flap hangs within clear opening",
        panel_inside_tunnel("outer_flap"),
        details="The outer flexible panel should clear the side liners and bottom threshold.",
    )
    ctx.check(
        "inner flap hangs within clear opening",
        panel_inside_tunnel("inner_flap"),
        details="The inner flexible panel should clear the side liners and bottom threshold.",
    )

    return ctx.report()


object_model = build_object_model()
