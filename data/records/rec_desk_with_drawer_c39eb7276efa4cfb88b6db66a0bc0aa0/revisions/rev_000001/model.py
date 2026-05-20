from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


PEDESTAL_CENTERS = (-0.58, 0.58)
DRAWER_LEVELS = (
    ("lower", 0.170),
    ("middle", 0.385),
    ("upper", 0.600),
)
PEDESTAL_DRAWER_TRAVEL = 0.250
FRIEZE_DRAWER_TRAVEL = 0.220


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="partners_desk")

    mahogany = model.material("mahogany", color=(0.30, 0.13, 0.055, 1.0))
    dark_mahogany = model.material("dark_mahogany", color=(0.18, 0.075, 0.035, 1.0))
    end_grain = model.material("end_grain", color=(0.24, 0.10, 0.045, 1.0))
    brass = model.material("brass", color=(0.86, 0.62, 0.24, 1.0))
    steel = model.material("dark_steel", color=(0.23, 0.24, 0.25, 1.0))
    leather = model.material("green_leather", color=(0.035, 0.16, 0.095, 1.0))

    body = model.part("desk_body")

    def add_body_box(name: str, size, xyz, material=mahogany) -> None:
        body.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)

    # A single broad work surface ties the two pedestal carcasses together.
    add_body_box("worktop", (1.88, 0.78, 0.055), (0.0, 0.0, 0.7325), mahogany)
    add_body_box("leather_inset", (1.44, 0.54, 0.006), (0.0, -0.005, 0.763), leather)

    ped_w = 0.48
    ped_d = 0.70
    panel_t = 0.035
    side_h = 0.710
    face_y = -0.358
    frame_depth = 0.026
    drawer_front_y = -0.383

    for p_index, px in enumerate(PEDESTAL_CENTERS):
        side_x = ped_w / 2.0 - panel_t / 2.0
        add_body_box(
            f"p{p_index}_outer_side",
            (panel_t, ped_d, side_h),
            (px - side_x, 0.0, side_h / 2.0),
            mahogany,
        )
        add_body_box(
            f"p{p_index}_inner_side",
            (panel_t, ped_d, side_h),
            (px + side_x, 0.0, side_h / 2.0),
            mahogany,
        )
        add_body_box(
            f"p{p_index}_back_panel",
            (ped_w, panel_t, side_h),
            (px, ped_d / 2.0 - panel_t / 2.0, side_h / 2.0),
            mahogany,
        )
        add_body_box(
            f"p{p_index}_plinth",
            (ped_w, ped_d, 0.070),
            (px, 0.0, 0.035),
            dark_mahogany,
        )
        add_body_box(
            f"p{p_index}_top_deck",
            (ped_w, ped_d, 0.035),
            (px, 0.0, 0.6925),
            mahogany,
        )

        # Front face frame: stiles and rails outline three separate drawer openings.
        for s_index, sx in enumerate((-side_x, side_x)):
            add_body_box(
                f"p{p_index}_front_stile_{s_index}",
                (0.050, frame_depth, 0.660),
                (px + sx, face_y, 0.370),
                end_grain,
            )
        for r_index, z in enumerate((0.078, 0.276, 0.492, 0.699)):
            add_body_box(
                f"p{p_index}_front_rail_{r_index}",
                (ped_w, frame_depth, 0.030),
                (px, face_y, z),
                end_grain,
            )

        # Fixed side-mounted guide rails for every pedestal drawer.  They are
        # slightly let into the side panels and face the moving rails on the drawers.
        for level_name, zc in DRAWER_LEVELS:
            rail_z = zc - 0.055
            for rail_index, sign in enumerate((-1.0, 1.0)):
                add_body_box(
                    f"p{p_index}_{level_name}_static_rail_{rail_index}",
                    (0.014, 0.500, 0.018),
                    (px + sign * 0.200, -0.075, rail_z),
                    steel,
                )

    # Central apron, rear modesty panel, and guide rails for the shared frieze drawer.
    add_body_box("rear_modesty_panel", (0.72, 0.035, 0.520), (0.0, 0.3325, 0.345), mahogany)
    add_body_box("front_frieze_rail", (0.72, frame_depth, 0.055), (0.0, face_y, 0.695), end_grain)
    add_body_box("front_knee_rail", (0.72, frame_depth, 0.040), (0.0, face_y, 0.545), end_grain)
    for rail_index, sx in enumerate((-0.3145, 0.3145)):
        add_body_box(
            f"frieze_static_rail_{rail_index}",
            (0.018, 0.500, 0.018),
            (sx, -0.075, 0.575),
            steel,
        )
    for bearer_index, sx in enumerate((-0.337, 0.337)):
        add_body_box(
            f"frieze_rail_bearer_{bearer_index}",
            (0.070, 0.500, 0.026),
            (sx, -0.075, 0.575),
            dark_mahogany,
        )

    def make_drawer(
        name: str,
        x: float,
        z: float,
        width: float,
        front_h: float,
        depth: float,
        rail_half_x: float,
        rail_z: float,
        travel: float,
    ) -> None:
        drawer = model.part(name)
        tray_w = width - 0.075
        tray_h = max(front_h - 0.055, 0.060)
        tray_y = 0.010 + depth / 2.0

        drawer.visual(
            Box((width, 0.024, front_h)),
            origin=Origin(),
            material=mahogany,
            name="front",
        )
        drawer.visual(
            Box((width - 0.070, 0.006, front_h - 0.055)),
            origin=Origin(xyz=(0.0, -0.014, 0.0)),
            material=dark_mahogany,
            name="inset_panel",
        )
        drawer.visual(
            Box((tray_w, depth, 0.016)),
            origin=Origin(xyz=(0.0, tray_y, -front_h / 2.0 + 0.040)),
            material=dark_mahogany,
            name="bottom",
        )
        for side_index, sx in enumerate((-1.0, 1.0)):
            drawer.visual(
                Box((0.018, depth, tray_h)),
                origin=Origin(xyz=(sx * (tray_w / 2.0 - 0.009), tray_y, -0.004)),
                material=mahogany,
                name=f"side_{side_index}",
            )
        drawer.visual(
            Box((tray_w, 0.018, tray_h)),
            origin=Origin(xyz=(0.0, 0.010 + depth - 0.009, -0.004)),
            material=mahogany,
            name="back",
        )
        for rail_index, sx in enumerate((-1.0, 1.0)):
            drawer.visual(
                Box((0.016, depth + 0.040, 0.018)),
                origin=Origin(
                    xyz=(
                        sx * rail_half_x,
                        0.010 + (depth + 0.040) / 2.0,
                        rail_z,
                    )
                ),
                material=steel,
                name=f"moving_rail_{rail_index}",
            )

        post_x = min(0.110, width * 0.24)
        for post_index, sx in enumerate((-post_x, post_x)):
            drawer.visual(
                Box((0.018, 0.030, 0.045)),
                origin=Origin(xyz=(sx, -0.025, 0.0)),
                material=brass,
                name=f"handle_post_{post_index}",
            )
        drawer.visual(
            Box((2.0 * post_x + 0.065, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, -0.049, 0.006)),
            material=brass,
            name="handle_bar",
        )

        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(x, drawer_front_y, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=85.0, velocity=0.35, lower=0.0, upper=travel),
        )

    for p_index, px in enumerate(PEDESTAL_CENTERS):
        for level_name, zc in DRAWER_LEVELS:
            make_drawer(
                f"pedestal_{p_index}_{level_name}_drawer",
                px,
                zc,
                0.360,
                0.180,
                0.540,
                0.185,
                -0.055,
                PEDESTAL_DRAWER_TRAVEL,
            )

    make_drawer(
        "frieze_drawer",
        0.0,
        0.620,
        0.600,
        0.120,
        0.500,
        0.2975,
        -0.045,
        FRIEZE_DRAWER_TRAVEL,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    drawer_names = [
        f"pedestal_{p_index}_{level_name}_drawer"
        for p_index in range(2)
        for level_name, _ in DRAWER_LEVELS
    ] + ["frieze_drawer"]
    joints = [object_model.get_articulation(f"body_to_{name}") for name in drawer_names]

    ctx.check(
        "seven independent drawer slides",
        len(joints) == 7
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )

    for p_index in range(2):
        for level_name, _ in DRAWER_LEVELS:
            name = f"pedestal_{p_index}_{level_name}_drawer"
            drawer = object_model.get_part(name)
            joint = object_model.get_articulation(f"body_to_{name}")
            static_rail = f"p{p_index}_{level_name}_static_rail_0"
            ctx.expect_overlap(
                drawer,
                "desk_body",
                axes="y",
                min_overlap=0.300,
                elem_a="moving_rail_0",
                elem_b=static_rail,
                name=f"{name} rail engaged closed",
            )
            rest_pos = ctx.part_world_position(drawer)
            with ctx.pose({joint: PEDESTAL_DRAWER_TRAVEL}):
                ctx.expect_overlap(
                    drawer,
                    "desk_body",
                    axes="y",
                    min_overlap=0.180,
                    elem_a="moving_rail_0",
                    elem_b=static_rail,
                    name=f"{name} rail retained open",
                )
                open_pos = ctx.part_world_position(drawer)
            ctx.check(
                f"{name} extends forward",
                rest_pos is not None
                and open_pos is not None
                and open_pos[1] < rest_pos[1] - 0.20,
                details=f"rest={rest_pos}, open={open_pos}",
            )

    frieze = object_model.get_part("frieze_drawer")
    frieze_joint = object_model.get_articulation("body_to_frieze_drawer")
    ctx.expect_overlap(
        frieze,
        "desk_body",
        axes="y",
        min_overlap=0.280,
        elem_a="moving_rail_0",
        elem_b="frieze_static_rail_0",
        name="frieze rail engaged closed",
    )
    rest_pos = ctx.part_world_position(frieze)
    with ctx.pose({frieze_joint: FRIEZE_DRAWER_TRAVEL}):
        ctx.expect_overlap(
            frieze,
            "desk_body",
            axes="y",
            min_overlap=0.160,
            elem_a="moving_rail_0",
            elem_b="frieze_static_rail_0",
            name="frieze rail retained open",
        )
        open_pos = ctx.part_world_position(frieze)
    ctx.check(
        "frieze drawer extends forward",
        rest_pos is not None and open_pos is not None and open_pos[1] < rest_pos[1] - 0.18,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
